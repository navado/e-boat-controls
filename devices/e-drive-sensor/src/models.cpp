#include <stdint.h>

#include "models.h"
#include "comms.h"

const uint8_t throttle_table[] = {245, 224, 185, 145, 110, 1, 110, 145, 185, 224, 254};
volatile panel_state_t panel_state = {};
volatile engine_state_t engine_state = {};
gps_state_t gps_state = {};
throttle_state_t throttle_state = {MODE_RPM, 0, 0, 0, 1, 1, 1, 0, 0};

const char * throttle_mode_names[] = {"RPM", "PWR", "SOG", "SOW", "RNG"};

void update_throttle_value(int val){
  if(val < 0 || val > 255){
    send_serial_dbg("Throttle value out of range", LOG_ERROR);
    return;
  }
  engine_state.throttle = (uint8_t)val;
}


on_off_t parse_on_off(String val){
  if(val == "on") return on;
  if(val == "off") return off;
  return error;
}

// ── PID ───────────────────────────────────────────────────────────────────────
float pid_compute(pid_state_t * pid, float target, float actual, float dt_s) {
  if (dt_s <= 0.0f) return pid->prev_output;
  float err = target - actual;
  pid->integral += err * dt_s;
  // Anti-windup
  if (pid->ki > 0.0f) {
    float i_max = pid->output_max / pid->ki;
    if (pid->integral >  i_max) pid->integral =  i_max;
    if (pid->integral < -i_max) pid->integral = -i_max;
  }
  float deriv = (err - pid->prev_error) / dt_s;
  float out   = pid->kp * err + pid->ki * pid->integral + pid->kd * deriv;
  pid->prev_error  = err;
  if (out > pid->output_max) out = pid->output_max;
  if (out < pid->output_min) out = pid->output_min;
  pid->prev_output = out;
  return out;
}

void pid_reset(pid_state_t * pid) {
  pid->integral    = 0.0f;
  pid->prev_error  = 0.0f;
  pid->prev_output = pid->output_min;
}

// ── GPS source arbitration ─────────────────────────────────────────────────────
gps_arb_t gps_arb = {};

uint8_t gps_arb_vote(gps_arb_t * a, unsigned long now_ms) {
  bool okT = a->last_T && (now_ms - a->last_T) < GPS_SRC_TIMEOUT_MS;
  bool okP = a->last_P && (now_ms - a->last_P) < GPS_SRC_TIMEOUT_MS;
  bool okS = a->last_S && (now_ms - a->last_S) < GPS_SRC_TIMEOUT_MS;
  if      (okT) a->active = GPS_SRC_THROTTLE;
  else if (okP) a->active = GPS_SRC_PANEL;
  else if (okS) a->active = GPS_SRC_SENSOR;
  else          a->active = GPS_SRC_NONE;
  return a->active;
}

// ── Propeller slip ────────────────────────────────────────────────────────────
int16_t calc_prop_slip(uint16_t rpm, uint16_t sow_kn10, uint16_t pitch_mm) {
  if (rpm == 0 || pitch_mm == 0) return 0;
  // Theoretical speed through water (m/s): RPM * pitch(mm) / 60000
  float v_theoretical = (float)rpm * (float)pitch_mm / 60000.0f;
  // Actual speed through water (m/s): knots * 0.5144 / 10
  float v_actual = (float)sow_kn10 * 0.05144f;
  float slip = (1.0f - v_actual / v_theoretical) * 100.0f;
  // Clamp to ±100 % and return as tenths of a percent
  if (slip >  100.0f) slip =  100.0f;
  if (slip < -100.0f) slip = -100.0f;
  return (int16_t)(slip * 10.0f);
}
