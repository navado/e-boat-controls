#include <Arduino.h>
#include <ui.h>
#include <pinout.h>
#include <models.h>
#include <buttons.h>
#include <comms.h>
#if defined(PANNEL_STM32)
  #include <HardwareTimer.h>
  static HardwareTimer _panelTimer(TIM2);
#else
  #include <TimeInterrupt.h>
#endif

// ── Optional NMEA 0183 GPS input on panel (USART3 PB11/PB10) ──────────────────
// Enable with -D NMEA0183_PANEL build flag.
// Panel then acts as GPS source on the internal bus via GPSRPT.
#if defined(NMEA0183_PANEL) && defined(PANNEL_STM32)
#include <HardwareSerial.h>
HardwareSerial NmeaSerial(NMEA_PANEL_RX, NMEA_PANEL_TX); // USART3
static String panel_nmea_buf;
static unsigned long last_gpsrpt_ms = 0;

static void read_panel_nmea() {
  while (NmeaSerial.available()) {
    char c = (char)NmeaSerial.read();
    if (c == '\n') {
      parse_nmea0183(panel_nmea_buf, &gps_state);
      panel_nmea_buf = "";
    } else if (c != '\r' && panel_nmea_buf.length() < 100) {
      panel_nmea_buf += c;
    }
  }
  // Broadcast GPS on internal bus at 1 Hz when we have a fix
  unsigned long now = millis();
  if ((gps_state.valid || gps_state.sow_valid) && (now - last_gpsrpt_ms >= 1000)) {
    send_gpsrpt(&Serial, GPS_SRC_PANEL, &gps_state);
    gps_arb.last_P = now;
    last_gpsrpt_ms = now;
  }
}
#else
static inline void read_panel_nmea() {}
#endif

void run_every_1s(){}
void run_every_100ms(){
  update_buttons();
}
void setup() {
  Serial.begin(115200);
#if defined(NMEA0183_PANEL) && defined(PANNEL_STM32)
  NmeaSerial.begin(9600);
#endif
  // Prepare keepalive LED
  pinMode(LED_BRD, OUTPUT);
  digitalWrite(LED_BRD, HIGH);
  setup_buttons();
  setup_screen();
  send_serial_dbg("PANEL READY", LOG_INFO);
#if defined(PANNEL_STM32)
  _panelTimer.setOverflow(100000, MICROSEC_FORMAT);  // 100 ms
  _panelTimer.attachInterrupt(run_every_100ms);
  _panelTimer.resume();
#else
  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt(run_every_100ms,100);
  TimeInterrupt.addInterrupt(run_every_1s,1000);
#endif
}


// ── Panel-local mode state (mirrors throttle_state.mode on THRINF receipt) ────
static throttle_mode_t panel_mode = MODE_RPM;

static void send_panmod() {
  char _msg[32];
  snprintf(_msg, sizeof(_msg), "%s,mode:%u", MSG_PAN_MODE, (uint8_t)panel_mode);
  send_msg(&Serial, _msg);
  send_serial_dbg(String("mode->") + throttle_mode_names[panel_mode], LOG_INFO);
}

void handle_throttle(){

  if (btn_a->state == btn_b->state)
    return;

  if (panel_state.power == 0 || panel_state.regen == 1)
    return;

  // Short press A: throttle / target step up
  if (BTN_PRESS(btn_a) && btn_a->short_press == 0){
    btn_a->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.speed = min(panel_state.speed + 1, THROTTLE_TABLE_SIZE - 1);
    PANEL_STATE_SET_CHANGED();
  }

  // Long press A: cycle control mode forward (RPM→PWR→SOG→SOW→RNG→RPM)
  if (BTN_LONG_PRESS(btn_a) && btn_a->long_press == 0){
    btn_a->long_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_mode = (throttle_mode_t)(((uint8_t)panel_mode + 1) % (uint8_t)MODE_COUNT);
    send_panmod();
  }

  // Short press B: throttle / target step down
  if (BTN_PRESS(btn_b) && btn_b->short_press == 0){
    btn_b->short_press = 1;
    btn_b->ht = BTN_MILLS;
    panel_state.speed = max(panel_state.speed - 1, 0);
    PANEL_STATE_SET_CHANGED();
  }

  // Long press B: cycle control mode backward (RPM←PWR←SOG←SOW←RNG←RPM)
  if (BTN_LONG_PRESS(btn_b) && btn_b->long_press == 0){
    btn_b->long_press = 1;
    btn_b->ht = BTN_MILLS;
    panel_mode = (throttle_mode_t)(
      ((uint8_t)panel_mode + (uint8_t)MODE_COUNT - 1) % (uint8_t)MODE_COUNT);
    send_panmod();
  }
}

void handle_state(){
  if (btn_c->state==btn_d->state) return;

  // BTN_C short: toggle engine power (guard: neutral speed, regen off)
  if (BTN_PRESS(btn_c) && btn_c->short_press == 0
      && panel_state.regen == 0 && panel_state.speed == SPD_NEUTRAL){
    btn_c->short_press = 1;
    btn_c->ht = BTN_MILLS;
    panel_state.power = !panel_state.power;
    if (panel_state.power == 0) panel_state.speed = SPD_NEUTRAL;
    PANEL_STATE_SET_CHANGED();
    send_serial_dbg(panel_state.power ? "PANEL: power ON" : "PANEL: power OFF", LOG_INFO);
  }

  // BTN_D short: toggle regen (guard: power on, neutral speed)
  if (BTN_PRESS(btn_d) && btn_d->short_press == 0
      && panel_state.power == 1 && panel_state.speed == SPD_NEUTRAL){
    btn_d->short_press = 1;
    btn_d->ht = BTN_MILLS;
    panel_state.regen = !panel_state.regen;
    PANEL_STATE_SET_CHANGED();
    send_serial_dbg(panel_state.regen ? "PANEL: regen ON" : "PANEL: regen OFF", LOG_INFO);
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
      #if (LOG_LEVEL == 0)   /* 0 = LOG_DEBUG */
       char _msg[128];
      snprintf(_msg, sizeof(_msg), "Checksum mismatch: %02X != %02X", checksum, (char)str_checksum_val);
      send_serial_dbg(_msg, LOG_ERROR);
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
      if (num_tokens >= 10) engine_state.curr_ma        = (uint16_t)parsed[9].toInt();
      if (num_tokens >= 11) engine_state.power_w        = (uint16_t)parsed[10].toInt();
      if (num_tokens >= 12) engine_state.water_kn10     = (uint16_t)parsed[11].toInt();
      if (num_tokens >= 13) engine_state.prop_slip_pct10 = (int16_t)parsed[12].toInt();
      engine_state.changed = 1;

    } else if (parsed[0] == MSG_THR_INFO && num_tokens >= 7) {
      // THRINF,T,mode,target,sog_kn10,sow_kn10,cog_deg — sync panel_mode from throttle
      throttle_state.mode       = (throttle_mode_t)parsed[2].toInt();
      throttle_state.target_val = (uint16_t)parsed[3].toInt();
      panel_mode = throttle_state.mode; // keep panel mode in sync
      if (gps_arb.active == GPS_SRC_NONE) {
        gps_state.sog_kn10 = (uint16_t)parsed[4].toInt();
        gps_state.sow_kn10 = (uint16_t)parsed[5].toInt();
        gps_state.cog_deg  = (uint16_t)parsed[6].toInt();
        gps_state.valid    = (gps_state.sog_kn10 > 0 || gps_state.cog_deg > 0);
      }

    } else if (parsed[0] == MSG_GPS_RPT) {
      // GPSRPT — GPS broadcast from throttle or sensor
      uint8_t prev_active = gps_arb.active;
      uint8_t src = GPS_SRC_NONE;
      gps_state_t remote = {};
      if (parse_gpsrpt(parsed, num_tokens, &src, &remote)) {
        unsigned long now = millis();
        if      (src == GPS_SRC_THROTTLE) gps_arb.last_T = now;
        else if (src == GPS_SRC_SENSOR)   gps_arb.last_S = now;
        gps_arb_vote(&gps_arb, now);
        if (gps_arb.active != prev_active) {
          char buf[40];
          snprintf(buf, sizeof(buf), "GPS src: %c->%c",
            prev_active ? (char)prev_active : '-',
            gps_arb.active ? (char)gps_arb.active : '-');
          send_serial_dbg(buf, LOG_WARN);
        }
#if !defined(NMEA0183_PANEL)
        if (gps_arb.active == src) gps_state = remote;
#endif
      }
    }
  }
}

void send_state(){
  // ENCMD carries mode so the sensor can forward it to the throttle device if present
  char _msg[128];
  snprintf(_msg, sizeof(_msg), "ENCMD,pow:%s,rev:%s,reg:%s,thr:%d,mode:%u",
    bool_to_on_of(panel_state.power).c_str(),
    bool_to_on_of(panel_state.speed < SPD_NEUTRAL).c_str(),
    bool_to_on_of(panel_state.regen).c_str(),
    throttle_table[panel_state.speed],
    (uint8_t)panel_mode
  );
  send_msg(&Serial, _msg);
  panel_state.changed = 0;
}

void loop()
{
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  read_panel_nmea();   // no-op unless NMEA0183_PANEL
  parse_serial_data();
  handle_throttle();
  handle_state();
  draw_screen();
  if(panel_state.changed)
    send_state();
}