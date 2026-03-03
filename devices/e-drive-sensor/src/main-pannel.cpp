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
      panel_state.speed = min(SPD_NEUTRAL + 4, THROTTLE_TABLE_SIZE - 1);
    } else { // Reverse to Neutral
      panel_state.speed = SPD_NEUTRAL;
    }
    PANEL_STATE_SET_CHANGED();
  }

  if (BTN_PRESS(btn_b) && btn_b->short_press == 0){
    btn_b->short_press = 1;
    btn_b->ht = BTN_MILLS;
    panel_state.speed = max(panel_state.speed - 1, 0);
    PANEL_STATE_SET_CHANGED();
  }

  if (BTN_LONG_PRESS(btn_b) && btn_b->long_press == 0){
    btn_b->long_press = 1;
    btn_b->ht = BTN_MILLS;
    if(panel_state.speed > SPD_NEUTRAL) { // Forward to Neutral
      panel_state.speed = max(panel_state.speed - 4, SPD_NEUTRAL);
    } else if (panel_state.speed == SPD_NEUTRAL){ // Neutral to Reverse
      panel_state.speed = max(SPD_NEUTRAL - 4, 0);
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
    btn_c->ht = BTN_MILLS;
    panel_state.power = !panel_state.power;
    if (panel_state.power == 0){
      panel_state.speed = SPD_NEUTRAL;
    }
    PANEL_STATE_SET_CHANGED();
  }
  if (BTN_PRESS(btn_d) && btn_d->short_press ==0 && panel_state.power == 1 && panel_state.speed == SPD_NEUTRAL){
    btn_d->short_press = 1;
    btn_d->ht = BTN_MILLS;
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
    if (msg_start < 0 || msg_end <= msg_start) return;
    String msg = data.substring(msg_start + 1, msg_end); // skip '$'
    str_checksum = data.substring(msg_end+1, data.length());
    checksum = msg_checksum(msg.c_str());
    long str_checksum_val = strtol(str_checksum.c_str(), NULL, 16);
    if(checksum != str_checksum_val){
      #if LOG_LEVEL==DEBUG
       char _msg[128];
      snprintf(_msg, sizeof(_msg), "Checksum mismatch: %02X != %02X", checksum, (char)str_checksum_val);
      send_serial_dbg(_msg,ERROR);
      #endif
      return;
    }
    String parsed[MAX_TOKENS];
    uint8_t num_tokens = tokenize(msg, ',', parsed, MAX_TOKENS);
    if (num_tokens < 2) return;

    if (parsed[0] == MSG_ENG_INFO && num_tokens >= 9) {
      // ENINF,T,RPM,POW,REV,REG,THR,VTH_MV,VCC_MV[,CURR_MA,POWER_W,WATER_KN10]
      engine_state.T          = parsed[1].toInt();
      engine_state.rpm        = parsed[2].toInt();
      engine_state.power      = parsed[3].toInt();
      engine_state.reverse    = parsed[4].toInt();
      engine_state.regen      = parsed[5].toInt();
      engine_state.throttle   = parsed[6].toInt();
      engine_state.throttle_val = parsed[7].toInt();
      engine_state.vcc48v     = (uint16_t)parsed[8].toInt();
      if (num_tokens >= 10) engine_state.curr_ma    = (uint16_t)parsed[9].toInt();
      if (num_tokens >= 11) engine_state.power_w    = (uint16_t)parsed[10].toInt();
      if (num_tokens >= 12) engine_state.water_kn10 = (uint16_t)parsed[11].toInt();
      engine_state.changed = 1;

    } else if (parsed[0] == MSG_THR_INFO && num_tokens >= 7) {
      // THRINF,T,mode,target,sog_kn10,sow_kn10,cog_deg
      throttle_state.mode      = (throttle_mode_t)parsed[2].toInt();
      throttle_state.target_val = (uint16_t)parsed[3].toInt();
      gps_state.sog_kn10       = (uint16_t)parsed[4].toInt();
      gps_state.sow_kn10       = (uint16_t)parsed[5].toInt();
      gps_state.cog_deg        = (uint16_t)parsed[6].toInt();
      gps_state.valid          = (gps_state.sog_kn10 > 0 || gps_state.cog_deg > 0);
    }
  }
}

void send_state(){
  char _msg[128];
  snprintf(_msg, sizeof(_msg), "ENCMD,pow:%s,rev:%s,reg:%s,thr:%d",
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