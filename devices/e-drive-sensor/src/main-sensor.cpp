#include <Arduino.h>
#include <TimeInterrupt.h>

#include <pinout.h>
#include <buttons.h>
#include <models.h>
#include <comms.h>


bool serial_sent = false;

void setup_rpm_counter(){
  pinMode(RPM_PIN, INPUT);
  TCCR1A=0; // setup timer-1
  TCCR1C=0;
  TIMSK1=0;
  GTCCR=0;
  TCCR1B=0b00000110; // falling edge
}

void run_every_1s(){
  engine_state.rpm = TCNT1 / 6; // 6 pulses per revolution
  TCNT1=0;

  // water_kn10 is updated by parse_bus_serial() when THRINF arrives from throttle device

  // Current sensing: ADC → mA
  // curr_mA = ADC_val * (Vref_mV / 1023) / CURR_MV_PER_AMP * 1000
  uint16_t adc_curr = analogRead(CURR_SENS_IN);
  engine_state.curr_ma = (uint16_t)((uint32_t)adc_curr * 5000 / 1023 * 1000 / CURR_MV_PER_AMP);

  // Power: P = V(mV) * I(mA) / 1e6  → W
  uint32_t v_mv = (uint32_t)engine_state.vcc48v;
  uint32_t i_ma = engine_state.curr_ma;
  engine_state.power_w = (uint16_t)((v_mv * i_ma) / 1000000UL);

  // Propeller slip (requires water speed from THRINF)
  engine_state.prop_slip_pct10 = calc_prop_slip(
    engine_state.rpm, engine_state.water_kn10, PROP_PITCH_MM);

  // Low-voltage warning (42V threshold on 48V nominal battery)
  if (engine_state.vcc48v > 0 && engine_state.vcc48v < 42000)
    send_serial_dbg("LOW VOLTAGE", LOG_WARN);
}

void run_every_100ms(){
  engine_state.throttle_val = analogRead(THROTTLE_IN);
  uint16_t raw_vcc = analogRead(VCC_SENS_IN);
  engine_state.vcc48v = (uint16_t)map(raw_vcc, 0, 1023, 0, 100000); // store as mV
  update_buttons();
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
}

void run_every_10s(){
    serial_sent = false;
}

void setup_timing_functions(){
  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt(run_every_100ms,100);
  TimeInterrupt.addInterrupt(run_every_1s,1000);
  TimeInterrupt.addInterrupt(run_every_10s,10*1000);
}

void set_state(){
  // TODO: implement guards for reverse and regen, not to switch them while throttle is actually powered

  digitalWrite(RELAY_LOCK, engine_state.power);
  digitalWrite(RELAY_REVERSE, engine_state.reverse);
  digitalWrite(RELAY_REGEN, engine_state.regen);
  digitalWrite(RELAY_SPARE, engine_state.spare);
  analogWrite(THROTTLE_OUT, engine_state.throttle);
}

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);
  pinMode(LED_BRD, OUTPUT);
  // Throttle
  pinMode(THROTTLE_IN, INPUT);
  pinMode(THROTTLE_OUT, OUTPUT);
  // VCC
  pinMode(VCC_SENS_IN, INPUT);
  // Timers/counters
  setup_rpm_counter();
  setup_timing_functions();
  // Relays
  pinMode(RELAY_LOCK, OUTPUT);
  pinMode(RELAY_REVERSE, OUTPUT);
  pinMode(RELAY_REGEN, OUTPUT);
  pinMode(RELAY_SPARE, OUTPUT);
  // Buttons
  setup_buttons();
  // LCD
  // setup_screen();
  set_state();
  char msg[] = "ENINF,st:STARTED";
  send_msg(&Serial, msg);
  send_serial_dbg("SENSOR READY", LOG_INFO);
}


bool handle_command(String token){
  String tok[4];
  uint8_t num_tokens = tokenize(token, ':', tok, 4);
  if(num_tokens != 2){
    return false;
  }
  cmd_t cmd = parse_cmd(tok[0]);
  on_off_t val = parse_on_off(tok[1]);
  int tv = 0;
  switch(cmd){
    case CMD_POWER:
      if(engine_state.power && engine_state.throttle > 1){
       send_serial_dbg("Cannot force power off when engine is running", LOG_WARN);
       break;
      }
      UPDATE_ON_OFF_FIELD(power, val);
      send_serial_dbg(engine_state.power ? "SENSOR: power ON" : "SENSOR: power OFF", LOG_INFO);
      break;
    case CMD_REVERSE:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        send_serial_dbg("Cannot force reverse when engine is running", LOG_WARN);
        break;
      } else if(engine_state.regen){
        send_serial_dbg("Cannot force reverse when regen is on", LOG_WARN);
        break;
      }
      UPDATE_ON_OFF_FIELD(reverse, val);
      break;
    case CMD_REGEN:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        send_serial_dbg("Cannot force regen when engine is running", LOG_WARN);
        break;
      } else if(engine_state.reverse){
        send_serial_dbg("Cannot force regen when reverse is on", LOG_WARN);
        break;
      }
      UPDATE_ON_OFF_FIELD(regen, val);
      break;
    case CMD_THROTTLE:
      if(engine_state.power==0){
        send_serial_dbg("Cannot set throttle when engine is off", LOG_WARN);
        break;
      }
      if(engine_state.regen){
        send_serial_dbg("Cannot set throttle when regen is on", LOG_WARN);
        break;
      }
      tv = tok[1].toInt();
      update_throttle_value(tv);
      break;
    case CMD_RESET:
      engine_state.power = 1;
      engine_state.reverse = 0;
      engine_state.regen = 0;
      engine_state.throttle = 0;
      break;
    case CMD_MODE:
      // Sensor stores mode for telemetry forwarding; control logic lives in throttle device
      if (tok[1].toInt() < MODE_COUNT)
        throttle_state.mode = (throttle_mode_t)tok[1].toInt();
      break;
    case CMD_TARGET:
      throttle_state.target_val = (uint16_t)tok[1].toInt();
      break;
    case CMD_UNKNOWN:
    default:
  #if defined(DEBUG)
      Serial.print("WARNING: Command not supported: ");
      Serial.println(token);
  #endif
    return false;
  }
  return true;
}

// ── Bus serial dispatcher ─────────────────────────────────────────────────────
// Reads one line and dispatches:
//   ENCMD / THRCMD  → command handler (motor control)
//   THRINF          → extract sow_kn10 / sog_kn10 for slip calculation
static String _bus_tokens[MAX_TOKENS];
void handle_bus_serial() {
  String line = Serial.readStringUntil('\n');
  uint8_t n = tokenize(line, ',', _bus_tokens, MAX_TOKENS);
  if (n == 0) return;
  const String & hdr = _bus_tokens[0];

  if (hdr == "$" MSG_ENG_CMD || hdr == "$" MSG_THR_CMD) {
    // Command tokens start at index 1; strip trailing '*XX' from last token
    // (checksum validation skipped here — sensor trusts bus devices)
    uint8_t good = 0;
    for (uint8_t i = 1; i < n; i++) {
      // Strip checksum from last token if present
      String tok = _bus_tokens[i];
      int star = tok.indexOf('*');
      if (star >= 0) tok = tok.substring(0, star);
      if (handle_command(tok)) good++;
    }
    (void)good;

  } else if (hdr == "$" MSG_THR_INFO && n >= 7) {
    // THRINF,T,mode,target,sog_kn10,sow_kn10,cog_deg[*CRC]
    // Fallback: use GPS from THRINF only when no GPSRPT source is active
    if (gps_arb.active == GPS_SRC_NONE) {
      String sow_str = _bus_tokens[5];
      int star = sow_str.indexOf('*');
      if (star >= 0) sow_str = sow_str.substring(0, star);
      engine_state.water_kn10  = (uint16_t)sow_str.toInt();
      gps_state.sog_kn10       = (uint16_t)_bus_tokens[4].toInt();
      gps_state.cog_deg        = (uint16_t)_bus_tokens[6].toInt();
      gps_state.valid          = 1;
      gps_state.sow_valid      = (engine_state.water_kn10 > 0);
    }

  } else if (hdr == "$" MSG_GPS_RPT) {
    // GPSRPT — GPS broadcast from throttle or panel; sensor trusts and applies it
    uint8_t src = GPS_SRC_NONE;
    gps_state_t remote = {};
    if (parse_gpsrpt(_bus_tokens, n, &src, &remote)) {
      uint8_t prev_active = gps_arb.active;
      unsigned long now = millis();
      if      (src == GPS_SRC_THROTTLE) gps_arb.last_T = now;
      else if (src == GPS_SRC_PANEL)    gps_arb.last_P = now;
      gps_arb_vote(&gps_arb, now);
      if (gps_arb.active != prev_active) {
        char buf[40];
        snprintf(buf, sizeof(buf), "GPS src: %c->%c",
          prev_active ? (char)prev_active : '-',
          gps_arb.active ? (char)gps_arb.active : '-');
        send_serial_dbg(buf, LOG_WARN);
      }
      if (gps_arb.active == src) {
        gps_state                = remote;
        engine_state.water_kn10  = remote.sow_kn10;
      }
    }
  }
}

void loop() {
  if(serial_available()){
    handle_bus_serial();
  }
  set_state();
  if (serial_sent) return;

  serial_sent = true;
  char msg[220];
  // ENINF,T,RPM,POW,REV,REG,THR,VTH_MV,VCC_MV,CURR_MA,POWER_W,WATER_KN10,SLIP_PCT10
  snprintf(msg, sizeof(msg),
    "ENINF,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d",
    millis(),
    engine_state.rpm,
    engine_state.power,
    engine_state.reverse,
    engine_state.regen,
    engine_state.throttle,
    (unsigned)map(engine_state.throttle_val, 0, 1023, 0, 5000),
    engine_state.vcc48v,
    engine_state.curr_ma,
    engine_state.power_w,
    engine_state.water_kn10,
    engine_state.prop_slip_pct10
  );
  while (Serial.available()) Serial.read();
  send_msg(&Serial, msg);
}

