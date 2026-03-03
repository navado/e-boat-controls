#include <stdint.h>

#include "models.h"
#include "comms.h"

const uint8_t throttle_table[] = {245, 224, 185, 145, 110, 1, 110, 145, 185, 224, 254};
volatile panel_state_t panel_state = {0};
volatile engine_state_t engine_state = {0};
gps_state_t gps_state = {0};
throttle_state_t throttle_state = {MODE_RPM, 0, 0, 0, 1, 1, 1, 0, 0};

const char * throttle_mode_names[] = {"RPM", "PWR", "SOG", "SOW", "RNG"};

void update_throttle_value(int val){
  if(val < 0 || val > 255){
    send_serial_dbg("Throttle value out of range", ERROR);
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
