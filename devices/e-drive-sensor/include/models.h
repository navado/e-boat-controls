#ifndef MODELS_H
#define MODELS_H

#include <Arduino.h>

typedef struct panel_state_ {
  union{
    struct{
      uint8_t power: 1; // 0 - OFF, 1 - ON
      uint8_t regen: 1; // 0 - OFF, 1 - ON
    };
    uint8_t mode:2;
  };
  uint8_t changed: 1;
  uint8_t spare: 1;
  uint8_t speed: 4; // 0 - 11 index in throttle_table
  uint16_t rpm :16;
} panel_state_t;

extern volatile panel_state_t panel_state;
#define PANEL_STATE_SET_CHANGED() panel_state.changed = 1;

typedef struct {
  uint8_t power:1;          // 0 - OFF, 1 - ON
  uint8_t reverse:1;        // 0 - OFF, 1 - ON
  uint8_t regen: 1;         // 0 - OFF, 1 - ON
  uint8_t changed: 1;
  uint8_t spare: 5;
  uint8_t throttle: 8;      // For analogWrite
  uint16_t throttle_val;    // As read from ADC
  uint16_t vcc48v;          // Battery voltage (mV from ENINF, raw ADC on sensor)
  uint16_t rpm;             // RPM
  unsigned long T;          // Timestamp
  uint16_t curr_ma;         // Motor current (mA)
  uint16_t power_w;         // Motor power (W)
  uint16_t water_kn10;      // Water speed (knots * 10, from NMEA via throttle device)
  int16_t  prop_slip_pct10; // Propeller slip (% * 10, signed; positive = slipping)
} engine_state_t;
extern volatile engine_state_t engine_state;

typedef enum {
  off = 0,
  on = 1,
  error = 2
} on_off_t;
on_off_t parse_on_off(String val);
#define UPDATE_ON_OFF_FIELD(f,v) if(v!=on_off_t::error) engine_state.f = v;

// ── Throttle table ────────────────────────────────────────────────────────────
#define THROTTLE_TABLE_SIZE 11
extern const uint8_t throttle_table[];
#define SPD_NEUTRAL 5

void update_throttle_value(int val);

// ── Throttle control modes ────────────────────────────────────────────────────
typedef enum : uint8_t {
  MODE_RPM   = 0,  // Direct RPM target (simple PID)
  MODE_POWER = 1,  // Constant power (Watts)
  MODE_SOG   = 2,  // Constant speed over ground (GPS)
  MODE_SOW   = 3,  // Constant speed over water (impeller)
  MODE_RANGE = 4,  // Maximum range (efficiency optimised)
  MODE_COUNT = 5
} throttle_mode_t;

extern const char * throttle_mode_names[];

// ── PID controller ────────────────────────────────────────────────────────────
typedef struct {
  float kp, ki, kd;
  float integral;
  float prev_error;
  float prev_output;
  float output_min, output_max;
} pid_state_t;

float pid_compute(pid_state_t * pid, float target, float actual, float dt_s);
void  pid_reset(pid_state_t * pid);

// ── GPS / navigation state ────────────────────────────────────────────────────
typedef struct {
  uint16_t sog_kn10;      // Speed over ground (knots * 10)
  uint16_t sow_kn10;      // Speed over water  (knots * 10)
  uint16_t cog_deg;       // Course over ground (degrees)
  uint8_t  valid     : 1; // GPS fix valid
  uint8_t  sow_valid : 1; // Water speed sensor valid
} gps_state_t;

extern gps_state_t gps_state;

// ── GPS source arbitration ─────────────────────────────────────────────────────
// Any node with a physical NMEA0183 port broadcasts GPSRPT on the shared bus.
// All nodes vote on the freshest / highest-priority source and detect disconnects.
//
// Priority (if multiple sources alive): Throttle ('T') > Panel ('P') > Sensor ('S')
// NOTE: Sensor NMEA is not hardware-supported (Nano Timer1 conflicts SoftwareSerial).
//       Sensor can receive GPS via GPSRPT from the bus but cannot originate it.
//
// A source is stale after GPS_SRC_TIMEOUT_MS ms of silence.
#define GPS_SRC_NONE     0
#define GPS_SRC_THROTTLE 'T'
#define GPS_SRC_PANEL    'P'
#define GPS_SRC_SENSOR   'S'
#define GPS_SRC_TIMEOUT_MS  5000UL

typedef struct {
  unsigned long last_T;  // millis() of last GPSRPT from throttle
  unsigned long last_P;  // millis() of last GPSRPT from panel
  unsigned long last_S;  // millis() of last GPSRPT from sensor
  uint8_t active;        // current winner: GPS_SRC_* or GPS_SRC_NONE
} gps_arb_t;

extern gps_arb_t gps_arb;

// Re-evaluate the active GPS source.  Call periodically; returns new active value.
uint8_t gps_arb_vote(gps_arb_t * a, unsigned long now_ms);

// ── Throttle device state ─────────────────────────────────────────────────────
typedef struct {
  throttle_mode_t mode;
  int16_t  raw_pos;        // Throttle position: 0=center, >0=fwd, <0=rev
  int16_t  center_offset;  // Calibrated center (encoder mode)
  uint16_t target_val;     // Control target: RPM | Watts | knots*10
  uint8_t  computed_thr;   // Output PWM 0-255 after mode/PID processing
  uint8_t  at_center  : 1; // In latch/dead-zone → neutral
  uint8_t  panel_on   : 1; // Panel power (managed by throttle device)
  uint8_t  changed    : 1;
  uint8_t  spare      : 5;
} throttle_state_t;

extern throttle_state_t throttle_state;

// ── Propeller slip ────────────────────────────────────────────────────────────
// Propeller pitch: distance the vessel would advance per revolution with zero slip.
// Set at build time via -D PROP_PITCH_MM=<value>.  Default 600 mm (typical small vessel).
#ifndef PROP_PITCH_MM
#define PROP_PITCH_MM 600
#endif

// Calculate apparent propeller slip.
// Returns slip * 10 (tenths of a percent, signed).
//   +ve = slipping (propeller churning more than advancing)
//   -ve = negative slip (e.g. sailing with engine assist in following current)
//   0   = no slip (or RPM == 0 → undefined)
//
// Formula:  slip% = (1 - v_actual / v_theoretical) * 100
//   v_theoretical (m/s) = rpm * pitch_mm / 60000
//   v_actual      (m/s) = sow_kn10 * 0.05144
int16_t calc_prop_slip(uint16_t rpm, uint16_t sow_kn10, uint16_t pitch_mm);

#endif
