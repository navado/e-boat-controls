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
    PANEL_STATE_SET_CHANGED();
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
    PANEL_STATE_SET_CHANGED();
  }

  if (BTN_PRESS(btn_b) && btn_b->short_press == 0){
    btn_b->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.speed = max(panel_state.speed - 1, 0);
    PANEL_STATE_SET_CHANGED();
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
    PANEL_STATE_SET_CHANGED();
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
    PANEL_STATE_SET_CHANGED();
  }
  if (BTN_PRESS(btn_d) && btn_d->short_press ==0 && panel_state.power == 1 && panel_state.speed == SPD_NEUTRAL){
    btn_d->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.regen = !panel_state.regen;
    PANEL_STATE_SET_CHANGED();
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
int msg_start;
int msg_end;
int len;
String str_checksum;
uint8_t checksum;
void display_serial_data(){
  uint8_t _x=64, _y = 8;
  _y = printKWLabel(_x, _y, "st: ", msg_start);
  _y = printKWLabel(_x, _y, "en: ", msg_end);
  _y = printKWLabel(_x, _y, "le: ", len);
  _y = printKWLabel(_x, _y, "sc: ", str_checksum);
  _y = printKWLabel(_x, _y, "ck: ", checksum);
}
void parse_serial_data(){
  if (Serial.available()){
    String data = Serial.readStringUntil('\n');
    len = data.length();
    msg_start = data.indexOf('$');
    msg_end = data.indexOf('*');
    String msg = data.substring(msg_start, msg_end);
    str_checksum = data.substring(msg_end+1, data.length());
    checksum = msg_checksum(msg.c_str());
    long str_checksum_val = strtol(str_checksum.c_str(), NULL, 16);
    if(checksum != str_checksum_val){
      #if LOG_LEVEL==DEBUG
       char _msg[128];
      sprintf(_msg, "Checksum missmatch: %02X != %02X", checksum, (char)str_checksum_val);
      send_serial_dbg(_msg,ERROR);
      #endif
    }
    String parsed[MAX_TOKENS];
    uint8_t num_tokens = tokenize(msg, ',', parsed, MAX_TOKENS);
    if(num_tokens == 0) return;
    if(parsed[0] != "ENINF") return;
    
    // ENINF,T,POW,REV,REG,THR,VTH,VCC

    engine_state.T = parsed[1].toInt();
    engine_state.rpm = parsed[2].toInt();
    engine_state.power = parsed[3].toInt();
    engine_state.reverse = parsed[4].toInt();
    engine_state.regen = parsed[5].toInt();
    engine_state.throttle = parsed[6].toInt();
    engine_state.throttle_val = parsed[7].toInt();
    engine_state.vcc48v = parsed[8].toInt();
    engine_state.changed = 1;
    Serial.print("$ENDBG,len:");
    Serial.print(msg.length());
    Serial.print(",rxc:");
    Serial.print(str_checksum);
    Serial.print(",c:");
    Serial.println(checksum, HEX);
  }
}

void send_state(){
  char _msg[128];
  sprintf(_msg,"ENCMD,pow:%s,rev:%s,reg:%s,thr:%d",
    bool_to_on_of(panel_state.power).c_str(),
    bool_to_on_of(panel_state.speed < SPD_NEUTRAL).c_str(),
    bool_to_on_of(panel_state.regen).c_str(),
    throttle_table[panel_state.speed]
  );
  send_msg(&Serial, _msg);
  panel_state.changed = 0;
}

void loop()
{
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  parse_serial_data();
  handle_throttle();
  handle_state();
  draw_screen();
  if(panel_state.changed)
    send_state();
}