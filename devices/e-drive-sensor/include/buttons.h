#ifndef BUTTONS_H
#define BUTTONS_H
#include <Arduino.h>
#include "pinout.h"

typedef struct
{
  union{
    struct {
      uint8_t state : 1; // 0 - released, 1 - pressed
      uint8_t ov: 1;
      uint8_t t: 6;
    };
    uint8_t input;
  };
  union{
  struct{
    uint8_t short_press : 1;
    uint8_t long_press : 1;
    uint8_t very_long_press : 1;
    uint8_t ht : 6;
  };
  uint8_t handle;
  };
} btn_state_t;


extern btn_state_t buttons_state[4];
extern btn_state_t * btn_a;
extern btn_state_t * btn_b;
extern btn_state_t * btn_c;
extern btn_state_t * btn_d;


#define BTN_PRESS_THRESHOLD 2
#define BTN_LONG_PRESS_THRESHOLD 40
#define BTN_MAX_DURATION 120

#define BTN_MILLS ((millis()/100) & 0x003F)
#define BTN_PRESS(btn)  (btn->state == 1 && BTN_MILLS - btn->t > BTN_PRESS_THRESHOLD)
#define BTN_LONG_PRESS(btn)  (btn->state == 1 && BTN_MILLS - btn->t > BTN_LONG_PRESS_THRESHOLD)
#define BTN_VERY_LONG_PRESS(btn)  (btn->state == 1 && btn->ov == 1)

void update_button_state(uint8_t index, uint8_t value);
void update_buttons();
#endif