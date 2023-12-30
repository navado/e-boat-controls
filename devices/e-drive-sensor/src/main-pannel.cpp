#include <Arduino.h>
#include <ui.h>
#include <pinout.h>
#include <models.h>
#include <buttons.h>
#include <comms.h>
#include <TimeInterrupt.h>

void run_every_1s(){}
void run_every_100ms(){
  update_buttons();
}
void setup() {
  Serial.begin(115200);
  // Prepare keepalive LED
  pinMode(LED_BRD, OUTPUT);
  digitalWrite(LED_BRD, HIGH);
  setup_buttons();
  setup_screen();
  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt(run_every_100ms,100);
  TimeInterrupt.addInterrupt(run_every_1s,1000);
}


void handle_throttle(){

  if (btn_a->state == btn_b->state)
    return;

  if (panel_state.power == 0 || panel_state.regen == 1)
    return;

  if (BTN_PRESS(btn_a) && btn_a->short_press == 0){
    btn_a->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.speed = min(panel_state.speed + 1, THROTTLE_TABLE_SIZE - 1);
  }

  if(BTN_LONG_PRESS(btn_a) && btn_a->long_press == 0){
    btn_a->long_press = 1;
    btn_a->ht = BTN_MILLS;
    if(panel_state.speed > SPD_NEUTRAL) { // Forward to faster
      panel_state.speed = min(panel_state.speed + 4, THROTTLE_TABLE_SIZE - 1);
    } else if (panel_state.speed == SPD_NEUTRAL){ // Neutral to Forward
      panel_state.speed = SPD_NEUTRAL;
    } else { // Reverse to Neutral
      panel_state.speed = SPD_NEUTRAL;
    }
  }

  if (BTN_PRESS(btn_b) && btn_b->short_press == 0){
    btn_b->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.speed = max(panel_state.speed - 1, 0);
  }

  if (BTN_LONG_PRESS(btn_b) && btn_b->long_press == 0){
    btn_b->long_press = 1;
    btn_a->ht = BTN_MILLS;
    if(panel_state.speed > SPD_NEUTRAL) { // Forward to Neutral
      panel_state.speed = max(panel_state.speed - 4, SPD_NEUTRAL);
    } else if (panel_state.speed == SPD_NEUTRAL){ // Neutral to Reverse
      panel_state.speed = SPD_NEUTRAL;
    } else { // Reverse to faster
      panel_state.speed = 0;
    }
  }
}

void handle_state(){
  if (btn_c->state==btn_d->state) return;
  
  if (BTN_PRESS(btn_c) && btn_c->short_press ==0 && panel_state.regen == 0 && panel_state.speed == SPD_NEUTRAL){
    btn_c->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.power = !panel_state.power;
    if (panel_state.power == 0){
      panel_state.speed = SPD_NEUTRAL;
    }
  }
  if (BTN_PRESS(btn_d) && btn_d->short_press ==0 && panel_state.power == 1 && panel_state.speed == SPD_NEUTRAL){
    btn_d->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.regen = !panel_state.regen;
  }
}

void serial_loopback(){
  if (Serial.available()){
    char c = Serial.read();
    if(c==',') c = '|';
    if(c==':') c = '-';
    Serial.write(c);
  }
}

void parse_serial_data(){
  if (Serial.available()){
    String data = Serial.readStringUntil('\n');
    String parsed[MAX_TOKENS];
    uint8_t num_tokens = tokenize(data, ',', parsed, MAX_TOKENS);
    if(num_tokens == 0) return;
    for (size_t i = 0; i < num_tokens; i++)
    {
      String tok[4];
      uint8_t num_tok = tokenize(parsed[i], ':', tok, 4);
      if(num_tok == 0) continue;
      if(i==0 && tok[0]!="T") continue; // not the right message
      if(tok[0]=="T") engine_state.T = tok[1].toInt();
      // t:132625,rpm:0,pow:off,rev:off,reg:off,thr:0,vth:0,vcc:0,btn0:off,btn1:off,btn2:off,btn3:on
      if(tok[0]=="rpm") engine_state.rpm = tok[1].toInt();
      if(tok[0]=="pow") UPDATE_ON_OFF_FIELD(power, parse_on_off(tok[1]));
      if(tok[0]=="rev") UPDATE_ON_OFF_FIELD(reverse, parse_on_off(tok[1]));
      if(tok[0]=="reg") UPDATE_ON_OFF_FIELD(regen, parse_on_off(tok[1]));
      if(tok[0]=="thr") engine_state.throttle = tok[1].toInt();
      if(tok[0]=="vth") engine_state.throttle_val = tok[1].toInt();
      if(tok[0]=="vcc") engine_state.vcc48v = tok[1].toInt();
    }
  }
}

void send_state(){
  Serial.print("cmd,");
  send_serial_field(&Serial, "pow", bool_to_on_of(panel_state.power));
  send_serial_field(&Serial, "rev", bool_to_on_of(panel_state.speed < SPD_NEUTRAL));
  send_serial_field(&Serial, "reg", bool_to_on_of(panel_state.regen));
  send_serial_field(&Serial, "thr", String(throttle_table[panel_state.speed]), true);
}

void loop()
{
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  parse_serial_data();
  panel_state_t old_state={0};
  memccpy(&old_state, &panel_state, 0, sizeof(panel_state_t));
  handle_throttle();
  handle_state();
  draw_screen();
  if(memcmp(&old_state, &panel_state, sizeof(panel_state_t))!=0)
    send_state();
}