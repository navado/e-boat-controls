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
  uint16_t water_kn10;      // Water speed (knots * 10, from impeller)
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

#endif
