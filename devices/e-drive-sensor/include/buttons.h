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
// Update an arbitrary btn_state_t from a raw pin value (1=pressed, 0=released).
// Useful for extra buttons outside the fixed buttons_state[] array.
void update_btn(btn_state_t * btn, uint8_t pin_value);
void update_buttons();
void setup_buttons();

// ── Quadrature encoder ────────────────────────────────────────────────────────
typedef struct {
  int16_t  count;        // Accumulated position (+ = CW, - = CCW)
  int8_t   delta;        // Last step direction: +1, -1, or 0
  uint8_t  last_a  : 1;
  uint8_t  last_b  : 1;
  uint8_t  changed : 1;  // Set when count changed; caller must clear
  // Integrated push-button
  uint8_t  btn_short : 1;
  uint8_t  btn_long  : 1;
  uint8_t  btn_t     : 6; // BTN_MILLS at last press start
} encoder_state_t;

// Poll quadrature encoder pins, update count/delta/changed.
// Call from timer ISR or tight loop (≥1 kHz recommended for fast rotation).
void update_encoder(encoder_state_t * enc, uint8_t pin_a, uint8_t pin_b);
// Poll encoder push-button (active-low INPUT_PULLUP).  Sets btn_short / btn_long.
void update_encoder_btn(encoder_state_t * enc, uint8_t btn_pin);

#endif