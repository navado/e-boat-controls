/**
 * main-throttle.cpp  –  Throttle controller firmware
 * Hardware: STM32F103C8 (BluePill)
 *
 * Bus topology
 *   Serial  (USART1 PA9/PA10, 115200 baud) – shared with panel and sensor
 *   Serial2 (USART2 PA2/PA3,    9600 baud) – NMEA0183 GPS / chart-plotter
 *
 * Message roles
 *   Sends  THRCMD  → sensor   (primary motor control every 100 ms or on change)
 *   Sends  THRINF  → all      (mode / GPS status at 1 Hz)
 *   Sends  PANCTL  → panel    (panel power on/off)
 *   Reads  ENINF   ← sensor   (RPM, current, voltage feedback for PID)
 *
 * Throttle input (compile-time selection)
 *   -D THROTTLE_POT : PA0 analog potentiometer, centre = ADC 512
 *   -D THROTTLE_ENC : PB0/PB1 quadrature encoder, centre = calibrated count
 *
 * Control modes (selected via mode-selector encoder)
 *   MODE_RPM   – pot position → target RPM, PID closes on engine_state.rpm
 *   MODE_POWER – pot position → target Watts, PID closes on engine_state.power_w
 *   MODE_SOG   – pot position → target SOG (kn*10), PID closes on gps_state.sog_kn10
 *   MODE_SOW   – pot position → target SOW (kn*10), PID closes on gps_state.sow_kn10
 *   MODE_RANGE – targets 30 % of MAX_RPM for maximum efficiency
 *
 * LED (common-cathode RGB, PWM)
 *   Solid RED           – engine off
 *   Fast-blink RED      – engine off but throttle not at centre (danger)
 *   Solid BLUE          – engine on, neutral
 *   Green + blink/RPM   – engine on, forward
 *   Yellow + blink/RPM  – engine on, reverse
 *
 * Buzzer
 *   Slow beep  – low voltage (< LOW_VOLT_MV)
 *   Fast beep  – overcurrent (> OVERCURR_MA)
 *   3 beeps    – sensor comms timeout
 */

#if defined(THROTTLE_STM32)

#include <Arduino.h>
#include <HardwareSerial.h>
#include "pinout.h"
#include "models.h"
#include "comms.h"
#include "buttons.h"
#include <TimerInterrupt_Generic.h>

// USART2 (PA2=TX, PA3=RX) — not pre-instantiated by the BluePill variant
HardwareSerial Serial2(PA3, PA2);

// ── Tuning constants ──────────────────────────────────────────────────────────
#define MAX_RPM           5000   // Rated max RPM
#define MAX_POWER_W       3000   // Rated max power
#define MAX_SOG_KN10      150    // Max SOG target (15.0 kn)

#define LOW_VOLT_MV      42000U  // Warn below 42 V
#define OVERCURR_MA      60000U  // Warn above 60 A

#define LATCH_ZONE_POT      40   // ADC counts either side of centre = neutral
#define LATCH_ZONE_ENC       2   // Encoder counts either side of zero = neutral

#define THRCMD_INTERVAL_MS  100  // Send THRCMD at ≤ 10 Hz
#define THRINF_INTERVAL_MS 1000  // Send THRINF at 1 Hz
#define SENSOR_TIMEOUT_MS  5000  // Sensor silent for 5 s = error

// LED blink periods (ms full cycle)
#define BLINK_DANGER   150
#define BLINK_SLOW    1000

// ── PID defaults (tune empirically per vessel) ────────────────────────────────
static pid_state_t pid_rpm   = {0.50f, 0.02f, 0.10f, 0, 0, 0,   1, 254};
static pid_state_t pid_power = {0.30f, 0.01f, 0.05f, 0, 0, 0,   1, 254};
static pid_state_t pid_sog   = {2.00f, 0.05f, 0.20f, 0, 0, 0,   1, 254};
static pid_state_t pid_sow   = {2.00f, 0.05f, 0.20f, 0, 0, 0,   1, 254};

// ── Encoder state ─────────────────────────────────────────────────────────────
static encoder_state_t mode_enc = {0};
#if defined(THROTTLE_ENC)
static encoder_state_t thr_enc  = {0};
#endif

// ── Engine / panel buttons ────────────────────────────────────────────────────
static btn_state_t _btn_engine = {};
static btn_state_t _btn_panel  = {};
static btn_state_t * btn_engine = &_btn_engine;
static btn_state_t * btn_panel  = &_btn_panel;

// ── Timestamps ────────────────────────────────────────────────────────────────
static unsigned long last_thrcmd_ms    = 0;
static unsigned long last_thrinf_ms    = 0;
static unsigned long last_eninf_ms     = 0;
static unsigned long last_pid_ms       = 0;
static unsigned long last_motor_out_ms = 0; // motor NMEA0183 + N2K output rate

// ── LED state ─────────────────────────────────────────────────────────────────
static uint8_t  led_r = 0, led_g = 0, led_b = 0;
static uint16_t led_blink_period = 0; // 0 = solid
static bool     led_on = true;
static unsigned long led_toggle_ms = 0;

static void led_set(uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms = 0) {
  led_r = r; led_g = g; led_b = b;
  led_blink_period = period_ms;
  led_on = true;
}

static void led_update() {
  uint8_t r = led_r, g = led_g, b = led_b;
  if (led_blink_period > 0) {
    unsigned long now = millis();
    if (now - led_toggle_ms >= led_blink_period / 2) {
      led_on = !led_on;
      led_toggle_ms = now;
    }
    if (!led_on) { r = 0; g = 0; b = 0; }
  }
  analogWrite(LED_R_PIN, r);
  analogWrite(LED_G_PIN, g);
  analogWrite(LED_B_PIN, b);
}

static void update_led_state() {
  bool power = engine_state.power;
  bool danger = !power && !throttle_state.at_center;
  if (danger) {
    led_set(255, 0, 0, BLINK_DANGER);
  } else if (!power) {
    led_set(255, 0, 0);
  } else if (throttle_state.at_center) {
    led_set(0, 0, 255);
  } else {
    uint16_t period = (uint16_t)(engine_state.rpm > 0
      ? constrain(map(engine_state.rpm, 0, MAX_RPM, BLINK_SLOW, 150), 150, BLINK_SLOW)
      : BLINK_SLOW);
    if (engine_state.reverse) led_set(255, 150, 0, period); // yellow
    else                      led_set(0, 255, 0,   period); // green
  }
}

// ── Buzzer ────────────────────────────────────────────────────────────────────
typedef enum { BZR_OFF, BZR_LOW_VOLT, BZR_OVERCURR, BZR_ERROR } buzzer_t;
static buzzer_t buzzer_alert = BZR_OFF;
static bool     buzzer_state  = false;
static unsigned long buzzer_next_ms = 0;

// Beep sequencer for 3-beep error pattern
static uint8_t error_beep_count = 0;

static void buzzer_tick() {
  if (buzzer_alert == BZR_OFF) { digitalWrite(BUZZER_PIN, LOW); return; }
  unsigned long now = millis();
  if (now < buzzer_next_ms) return;

  uint16_t on_ms, off_ms;
  if (buzzer_alert == BZR_OVERCURR) { on_ms = 80;  off_ms = 80;  }
  else if (buzzer_alert == BZR_ERROR) {
    on_ms = 80; off_ms = 80;
    if (!buzzer_state && error_beep_count >= 6) { // 3 full beeps
      buzzer_next_ms = now + 800; error_beep_count = 0; return;
    }
    error_beep_count++;
  } else                            { on_ms = 200; off_ms = 800; } // low volt

  buzzer_state = !buzzer_state;
  digitalWrite(BUZZER_PIN, buzzer_state ? HIGH : LOW);
  buzzer_next_ms = now + (buzzer_state ? on_ms : off_ms);
}

static void update_buzzer_alert() {
  uint32_t vcc_mv  = (uint32_t)engine_state.vcc48v;
  uint32_t curr_ma = (uint32_t)engine_state.curr_ma;
  bool     timeout = (last_eninf_ms > 0 && millis() - last_eninf_ms > SENSOR_TIMEOUT_MS);

  if (timeout)                               buzzer_alert = BZR_ERROR;
  else if (curr_ma > OVERCURR_MA)            buzzer_alert = BZR_OVERCURR;
  else if (vcc_mv < LOW_VOLT_MV && engine_state.power) buzzer_alert = BZR_LOW_VOLT;
  else                                       buzzer_alert = BZR_OFF;
}

// ── Throttle position reading ─────────────────────────────────────────────────
#if defined(THROTTLE_POT)
static void read_throttle_position() {
  int raw = analogRead(THROTTLE_POT_PIN); // 0–1023
  throttle_state.raw_pos = (int16_t)(raw - 512) - throttle_state.center_offset;
  throttle_state.at_center = (abs(throttle_state.raw_pos) <= LATCH_ZONE_POT);
}
#elif defined(THROTTLE_ENC)
static void read_throttle_position() {
  update_encoder(&thr_enc, THROTTLE_ENC_A, THROTTLE_ENC_B);
  throttle_state.raw_pos = thr_enc.count - throttle_state.center_offset;
  throttle_state.at_center = (abs(throttle_state.raw_pos) <= LATCH_ZONE_ENC);
}
#else
// Fallback: pot mode
static void read_throttle_position() {
  int raw = analogRead(THROTTLE_POT_PIN);
  throttle_state.raw_pos = (int16_t)(raw - 512) - throttle_state.center_offset;
  throttle_state.at_center = (abs(throttle_state.raw_pos) <= LATCH_ZONE_POT);
}
#endif

// Map |raw_pos| to a throttle PWM value (0=neutral … 254=max).
// Direction is encoded in the sign of raw_pos; this returns magnitude only.
static uint8_t pos_to_pwm(int16_t raw_pos) {
  if (throttle_state.at_center) return 1;
#if defined(THROTTLE_ENC)
  int16_t max_pos = 100; // encoder counts at full travel
  int16_t latch   = LATCH_ZONE_ENC;
#else
  int16_t max_pos = 512 - LATCH_ZONE_POT;
  int16_t latch   = LATCH_ZONE_POT;
#endif
  int16_t abs_pos = abs(raw_pos);
  abs_pos = constrain(abs_pos, latch, max_pos);
  // Map position to throttle table index offset from neutral (0–5)
  uint8_t idx = (uint8_t)map(abs_pos, latch, max_pos, 0, 5);
  uint8_t tbl_idx = (raw_pos > 0) ? (SPD_NEUTRAL + idx) : (SPD_NEUTRAL - idx);
  tbl_idx = constrain(tbl_idx, 0, (int)(THROTTLE_TABLE_SIZE - 1));
  return throttle_table[tbl_idx];
}

// ── Control loop ──────────────────────────────────────────────────────────────
static void run_control_loop() {
  unsigned long now = millis();
  float dt_s = (float)(now - last_pid_ms) / 1000.0f;
  last_pid_ms = now;

  if (throttle_state.at_center) {
    throttle_state.computed_thr = 1; return;
  }

#if defined(THROTTLE_ENC)
  int16_t max_pos = 100;
  int16_t latch   = LATCH_ZONE_ENC;
#else
  int16_t max_pos = 512 - LATCH_ZONE_POT;
  int16_t latch   = LATCH_ZONE_POT;
#endif
  uint16_t abs_pos = (uint16_t)constrain(abs(throttle_state.raw_pos), latch, max_pos);
  uint8_t  base    = pos_to_pwm(throttle_state.raw_pos);

  switch (throttle_state.mode) {
    case MODE_RPM: {
      throttle_state.target_val = (uint16_t)map(abs_pos, latch, max_pos, 0, MAX_RPM);
      float out = pid_compute(&pid_rpm,
        (float)throttle_state.target_val, (float)engine_state.rpm, dt_s);
      throttle_state.computed_thr = (uint8_t)constrain((int)out, 1, 254);
      break;
    }
    case MODE_POWER: {
      throttle_state.target_val = (uint16_t)map(abs_pos, latch, max_pos, 0, MAX_POWER_W);
      float out = pid_compute(&pid_power,
        (float)throttle_state.target_val, (float)engine_state.power_w, dt_s);
      throttle_state.computed_thr = (uint8_t)constrain((int)out, 1, 254);
      break;
    }
    case MODE_SOG: {
      throttle_state.target_val = (uint16_t)map(abs_pos, latch, max_pos, 0, MAX_SOG_KN10);
      if (gps_state.valid) {
        float out = pid_compute(&pid_sog,
          (float)throttle_state.target_val, (float)gps_state.sog_kn10, dt_s);
        throttle_state.computed_thr = (uint8_t)constrain((int)out, 1, 254);
      } else {
        throttle_state.computed_thr = base; // no GPS: direct map
      }
      break;
    }
    case MODE_SOW: {
      throttle_state.target_val = (uint16_t)map(abs_pos, latch, max_pos, 0, MAX_SOG_KN10);
      if (gps_state.sow_valid) {
        float out = pid_compute(&pid_sow,
          (float)throttle_state.target_val, (float)gps_state.sow_kn10, dt_s);
        throttle_state.computed_thr = (uint8_t)constrain((int)out, 1, 254);
      } else {
        throttle_state.computed_thr = base;
      }
      break;
    }
    case MODE_RANGE: {
      // Target efficiency sweet-spot: 30 % of max RPM
      throttle_state.target_val = MAX_RPM * 3 / 10;
      float out = pid_compute(&pid_rpm,
        (float)throttle_state.target_val, (float)engine_state.rpm, dt_s);
      throttle_state.computed_thr = (uint8_t)constrain((int)out, 1, 180); // cap 70 %
      break;
    }
    default:
      throttle_state.computed_thr = base;
  }
}

// ── Mode selector encoder ─────────────────────────────────────────────────────
static void handle_mode_encoder() {
  update_encoder(&mode_enc, MODE_ENC_A, MODE_ENC_B);
  update_encoder_btn(&mode_enc, MODE_ENC_BTN);

  if (mode_enc.changed) {
    mode_enc.changed = 0;
    int8_t new_mode = (int8_t)throttle_state.mode + mode_enc.delta;
    new_mode = (new_mode % (int8_t)MODE_COUNT + MODE_COUNT) % MODE_COUNT;
    throttle_state.mode = (throttle_mode_t)new_mode;
    // Reset all PIDs on mode change to avoid integral windup carry-over
    pid_reset(&pid_rpm);
    pid_reset(&pid_power);
    pid_reset(&pid_sog);
    pid_reset(&pid_sow);
    throttle_state.changed = 1;
    send_serial_dbg(throttle_mode_names[throttle_state.mode], LOG_INFO);
  }

  // Button press: set throttle centre calibration (encoder mode)
  if (mode_enc.btn_short) {
    mode_enc.btn_short = 0;
#if defined(THROTTLE_ENC)
    throttle_state.center_offset = thr_enc.count; // latch current position as zero
#endif
  }
}

// ── NMEA 0183 GPS input (Serial2 RX, PA3) ─────────────────────────────────────
// Enabled when NMEA0183_THROTTLE is defined (default unless another node has NMEA).
// Serial2 TX (PA2) always outputs motor sentences regardless of this flag.
static String nmea_buf;

static void read_nmea_gps() {
#if defined(NMEA0183_THROTTLE)
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      parse_nmea0183(nmea_buf, &gps_state); // updates gps_state; broadcast deferred to THRINF tick
      nmea_buf = "";
    } else if (c != '\r' && nmea_buf.length() < 100) {
      nmea_buf += c;
    }
  }
#endif
}

// ── NMEA 2000 (CAN bus) ───────────────────────────────────────────────────────
// Requires external CAN transceiver (SN65HVD230 or equivalent) on PA11/PA12
// and the NMEA2000 + NMEA2000_stm32 libraries.
// Enable with -D NMEA2000 build flag.
//
// Receives:  PGN 128259 — Speed Through Water
//            PGN 129026 — COG & SOG Rapid Update
// Transmits: PGN 127488 — Engine Parameters Rapid Update (RPM)
//            PGN 127508 — Battery Status (voltage, current)
#if defined(NMEA2000)
#include <NMEA2000_CAN.h>   // platform CAN driver
#include <N2kMessages.h>

static void nmea2000_msg_handler(const tN2kMsg & msg) {
  switch (msg.PGN) {
    case 128259: { // Speed Through Water
      double sow_ms;
      if (ParseN2kBoatSpeed(msg, sow_ms)) {
        // 1 m/s = 19.438 kn*10
        gps_state.sow_kn10  = (uint16_t)(sow_ms * 19.438f);
        gps_state.sow_valid = 1;
      }
      break;
    }
    case 129026: { // COG & SOG Rapid
      uint8_t sid; tN2kHeadingReference ref; double cog_rad, sog_ms;
      if (ParseN2kCOGSOGRapid(msg, sid, ref, cog_rad, sog_ms)) {
        gps_state.sog_kn10 = (uint16_t)(sog_ms * 19.438f);
        gps_state.cog_deg  = (uint16_t)(cog_rad * 180.0f / PI);
        gps_state.valid    = 1;
      }
      break;
    }
    default: break;
  }
}

// Transmit motor telemetry PGNs (call at ~1 Hz).
static void send_motor_n2k() {
  if (last_eninf_ms == 0) return; // no sensor data yet
  tN2kMsg msg;
  // PGN 127488 — Engine Parameters Rapid Update (instance 0 = port/only engine)
  SetN2kEngineParamRapid(msg, 0, (double)engine_state.rpm);
  NMEA2000.SendMsg(msg);
  // PGN 127508 — Battery Status (instance 0 = main 48 V bus)
  SetN2kBatStatus(msg, 0,
    (double)engine_state.vcc48v  / 1000.0,  // mV → V
    (double)engine_state.curr_ma / 1000.0); // mA → A
  NMEA2000.SendMsg(msg);
}

static void setup_nmea2000() {
  NMEA2000.SetProductInformation("00000001", 100, "e-boat-throttle", "1.0", "1.0");
  NMEA2000.SetDeviceInformation(1, 50, 20, 2048); // unique, function, class, manufacturer
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  NMEA2000.SetMsgHandler(nmea2000_msg_handler);
  NMEA2000.Open();
}

static void read_nmea2000() {
  NMEA2000.ParseMessages();
}
#else
static inline void setup_nmea2000()  {}
static inline void read_nmea2000()   {}
static inline void send_motor_n2k()  {}
#endif // NMEA2000

// ── Bus message parser ────────────────────────────────────────────────────────
static void parse_bus_data() {
  if (!Serial.available()) return;
  String data = Serial.readStringUntil('\n');
  int msg_start = data.indexOf('$');
  int msg_end   = data.indexOf('*');
  if (msg_start < 0 || msg_end <= msg_start) return;

  String msg = data.substring(msg_start + 1, msg_end);
  char calc_cs = msg_checksum(msg.c_str());
  long rx_cs   = strtol(data.substring(msg_end + 1).c_str(), NULL, 16);
  if (calc_cs != (char)rx_cs) return;

  String parsed[MAX_TOKENS];
  uint8_t n = tokenize(msg, ',', parsed, MAX_TOKENS);
  if (n < 2) return;

  if (parsed[0] == MSG_ENG_INFO && n >= 9) {
    // ENINF — engine telemetry from sensor
    engine_state.T          = parsed[1].toInt();
    engine_state.rpm        = parsed[2].toInt();
    engine_state.power      = parsed[3].toInt();
    engine_state.reverse    = parsed[4].toInt();
    engine_state.regen      = parsed[5].toInt();
    engine_state.throttle   = parsed[6].toInt();
    engine_state.throttle_val = parsed[7].toInt();
    engine_state.vcc48v     = (uint16_t)parsed[8].toInt();
    if (n >= 10) engine_state.curr_ma         = (uint16_t)parsed[9].toInt();
    if (n >= 11) engine_state.power_w         = (uint16_t)parsed[10].toInt();
    if (n >= 12) engine_state.water_kn10      = (uint16_t)parsed[11].toInt();
    if (n >= 13) engine_state.prop_slip_pct10 = (int16_t)parsed[12].toInt();
    last_eninf_ms = millis();

  } else if (parsed[0] == MSG_GPS_RPT) {
    // GPSRPT — GPS broadcast from another node (panel or sensor)
    uint8_t src = GPS_SRC_NONE;
    gps_state_t remote = {};
    if (parse_gpsrpt(parsed, n, &src, &remote)) {
      unsigned long now = millis();
      if      (src == GPS_SRC_PANEL)  gps_arb.last_P = now;
      else if (src == GPS_SRC_SENSOR) gps_arb.last_S = now;
      gps_arb_vote(&gps_arb, now);
#if !defined(NMEA0183_THROTTLE)
      // No local NMEA: accept GPS data from the active remote source
      if (gps_arb.active == src) gps_state = remote;
#endif
    }
  }
}

// ── Engine and panel buttons ──────────────────────────────────────────────────
static void handle_engine_button() {
  update_btn(btn_engine, !digitalRead(BTN_ENGINE_PIN)); // active-low
  if (BTN_PRESS(btn_engine) && btn_engine->short_press == 0) {
    btn_engine->short_press = 1;
    btn_engine->ht = BTN_MILLS;
    if (!engine_state.power) {
      if (throttle_state.at_center) {
        engine_state.power = 1;
        throttle_state.changed = 1;
      } else {
        // Throttle not at centre: warn and refuse
        buzzer_alert = BZR_ERROR;
      }
    } else {
      if (throttle_state.at_center) {
        engine_state.power = 0;
        throttle_state.changed = 1;
      }
    }
  }
}

static void handle_panel_button() {
  update_btn(btn_panel, !digitalRead(BTN_PANEL_PIN));
  // Long-press to toggle panel power; interlock prevents off while engine runs
  if (BTN_LONG_PRESS(btn_panel) && btn_panel->long_press == 0) {
    btn_panel->long_press = 1;
    btn_panel->ht = BTN_MILLS;
    if (throttle_state.panel_on && engine_state.power) {
      buzzer_alert = BZR_ERROR; // interlock: cannot switch off while engine running
      return;
    }
    throttle_state.panel_on = !throttle_state.panel_on;
    char _msg[64];
    snprintf(_msg, sizeof(_msg), "%s,pow:%s",
      MSG_PAN_CTL, bool_to_on_of(throttle_state.panel_on).c_str());
    send_msg(&Serial, _msg);
  }
}

// ── Message senders ───────────────────────────────────────────────────────────
static void send_thrcmd() {
  char _msg[128];
  bool rev = (throttle_state.raw_pos < 0) && !throttle_state.at_center;
  snprintf(_msg, sizeof(_msg),
    "%s,pow:%s,rev:%s,reg:off,thr:%u,mode:%u,target:%u",
    MSG_THR_CMD,
    bool_to_on_of(engine_state.power).c_str(),
    bool_to_on_of(rev).c_str(),
    throttle_state.computed_thr,
    (uint8_t)throttle_state.mode,
    throttle_state.target_val);
  send_msg(&Serial, _msg);
  throttle_state.changed = 0;
  last_thrcmd_ms = millis();
}

static void send_thrinf() {
  unsigned long now = millis();
  char _msg[128];
  snprintf(_msg, sizeof(_msg),
    "%s,%lu,%u,%u,%u,%u,%u",
    MSG_THR_INFO, now,
    (uint8_t)throttle_state.mode,
    throttle_state.target_val,
    gps_state.sog_kn10,
    gps_state.sow_kn10,
    gps_state.cog_deg);
  send_msg(&Serial, _msg);

  // Broadcast GPS on internal bus so panel/sensor can use it if throttle has NMEA
#if defined(NMEA0183_THROTTLE)
  if (gps_state.valid || gps_state.sow_valid) {
    send_gpsrpt(&Serial, GPS_SRC_THROTTLE, &gps_state);
    gps_arb.last_T = now;
  }
#endif

  last_thrinf_ms = now;
}

// ── Arduino setup / loop ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);  // shared bus
  Serial2.begin(9600);   // NMEA 0183 GPS

  pinMode(LED_BRD,       OUTPUT);
  pinMode(LED_R_PIN,     OUTPUT);
  pinMode(LED_G_PIN,     OUTPUT);
  pinMode(LED_B_PIN,     OUTPUT);
  pinMode(BUZZER_PIN,    OUTPUT);
  pinMode(BTN_ENGINE_PIN, INPUT_PULLUP);
  pinMode(BTN_PANEL_PIN,  INPUT_PULLUP);
  pinMode(MODE_ENC_A,     INPUT_PULLUP);
  pinMode(MODE_ENC_B,     INPUT_PULLUP);
  pinMode(MODE_ENC_BTN,   INPUT_PULLUP);

#if defined(THROTTLE_POT)
  // PA0 is default analog; no explicit pinMode needed on STM32
#elif defined(THROTTLE_ENC)
  pinMode(THROTTLE_ENC_A, INPUT_PULLUP);
  pinMode(THROTTLE_ENC_B, INPUT_PULLUP);
#else
  // pot fallback
#endif

  setup_nmea2000(); // no-op unless -D NMEA2000

  // Startup blink: solid blue for 500 ms
  analogWrite(LED_B_PIN, 200);
  delay(500);
  analogWrite(LED_B_PIN, 0);

  last_pid_ms = millis();
  char msg[] = MSG_THR_INFO ",0,0,0,0,0,0";
  send_msg(&Serial, msg);
}

void loop() {
  // 1. Read speed/position data — NMEA 0183 on Serial2 and/or NMEA 2000 via CAN
  read_nmea_gps();
  read_nmea2000();

  // 2. Read bus (collect ENINF feedback from sensor)
  parse_bus_data();

  // 3. Read throttle position
  read_throttle_position();

  // 4. Mode encoder
  handle_mode_encoder();

  // 5. Buttons
  handle_engine_button();
  handle_panel_button();

  // 6. Control loop (PID / direct map)
  run_control_loop();

  // 7. Send THRCMD: immediately on state change, otherwise at fixed interval
  unsigned long now = millis();
  bool interval_due = (now - last_thrcmd_ms >= THRCMD_INTERVAL_MS);
  if (throttle_state.changed || interval_due) {
    send_thrcmd();
  }

  // 8. Send THRINF at 1 Hz
  if (now - last_thrinf_ms >= THRINF_INTERVAL_MS) {
    send_thrinf();
  }

  // 9. Motor telemetry output — NMEA 0183 (Serial2 TX) and NMEA 2000 (CAN) at 1 Hz
  if (now - last_motor_out_ms >= 1000) {
    send_nmea0183_rpm(&Serial2, engine_state.rpm);
    send_nmea0183_xdr(&Serial2, engine_state.vcc48v, engine_state.curr_ma, engine_state.power_w);
    send_motor_n2k();
    gps_arb_vote(&gps_arb, now); // periodic source staleness check
    last_motor_out_ms = now;
  }

  // 10. LED and buzzer
  update_led_state();
  led_update();
  update_buzzer_alert();
  buzzer_tick();

  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  delay(10); // ~100 Hz loop
}

#endif // THROTTLE_STM32
