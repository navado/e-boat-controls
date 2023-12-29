#include <stdint.h>

#include "models.h"
#include "comms.h"

const uint8_t throttle_table[] = {245, 224, 185, 145, 110, 1, 110, 145, 185, 224, 254};
volatile panel_state_t panel_state = {0};
volatile engine_state_t engine_state = {0};

void update_throttle_value(int val){
  if(val < 0 || val > 255){
    send_serial_error("ERROR: Throttle value out of range");
    return;
  }
  engine_state.throttle = (uint8_t)val;
}
