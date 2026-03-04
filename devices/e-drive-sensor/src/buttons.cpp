#include <buttons.h>
#include <Arduino.h>

btn_state_t buttons_state[4] = {0};
btn_state_t * btn_a = &buttons_state[0];
btn_state_t * btn_b = &buttons_state[1];
btn_state_t * btn_c = &buttons_state[2];
btn_state_t * btn_d = &buttons_state[3];

void _update_button_state(btn_state_t * btn, uint8_t value){
  if(BTN_MILLS - btn->t > BTN_MAX_DURATION){
    btn->ov = 1; // very long press
  }
  if (btn->state != value){
    btn->t = BTN_MILLS; // Store change time
  }
  btn->state = value;
  if (btn->state == 0){
    btn->input = 0;
    btn->handle = 0;
  }
}

void update_button_state(uint8_t index, uint8_t value){
  btn_state_t * btn = &buttons_state[index];
  _update_button_state(btn, value);
}

void update_buttons(){
#if !defined(THROTTLE_STM32)
  update_button_state(0, digitalRead(BTN_A));
  update_button_state(1, digitalRead(BTN_B));
  update_button_state(2, digitalRead(BTN_C));
  update_button_state(3, digitalRead(BTN_D));
#endif
}

// ── Generic single-button update (for extra buttons not in buttons_state[]) ──
void update_btn(btn_state_t * btn, uint8_t pin_value) {
  _update_button_state(btn, pin_value);
}

// ── Quadrature encoder ────────────────────────────────────────────────────────
// Simple X1 decode: count changes only on A edge.
void update_encoder(encoder_state_t * enc, uint8_t pin_a, uint8_t pin_b) {
  uint8_t a = digitalRead(pin_a) ? 1 : 0;
  uint8_t b = digitalRead(pin_b) ? 1 : 0;
  enc->delta = 0;
  if (a != enc->last_a) {
    enc->last_a = a;
    if (a == b) { enc->count++; enc->delta =  1; }
    else        { enc->count--; enc->delta = -1; }
    enc->changed = 1;
  }
  enc->last_b = b;
}

// Poll encoder push-button (active-low).  Sets btn_short on short press.
void update_encoder_btn(encoder_state_t * enc, uint8_t btn_pin) {
  uint8_t pressed = !digitalRead(btn_pin); // active-low
  uint8_t now_t = BTN_MILLS;
  if (pressed) {
    if (enc->btn_t == 0) enc->btn_t = now_t;
    uint8_t held = now_t - enc->btn_t;
    if (held > BTN_LONG_PRESS_THRESHOLD && !enc->btn_long) {
      enc->btn_long = 1;
    }
  } else {
    if (enc->btn_t != 0) {
      uint8_t held = now_t - enc->btn_t;
      if (held > BTN_PRESS_THRESHOLD && !enc->btn_long) enc->btn_short = 1;
      enc->btn_t = 0;
      enc->btn_long = 0;
    }
  }
}

void setup_buttons(){
#if !defined(THROTTLE_STM32)
  // Buttons
  pinMode(BTN_A, INPUT);
  pinMode(BTN_B, INPUT);
  pinMode(BTN_C, INPUT);
  pinMode(BTN_D, INPUT);
#endif
#if defined(PANNEL_STM32)
  attachInterrupt(BTN_A, [](){ // Throttle UP
    update_button_state(0, digitalRead(BTN_A));
  }, CHANGE);
  attachInterrupt(BTN_B, [](){ // Throttle DOWN
    update_button_state(1, digitalRead(BTN_B));
  }, CHANGE);
  attachInterrupt(BTN_C, [](){ // Reverse
    update_button_state(2, digitalRead(BTN_C));
  }, CHANGE);
  attachInterrupt(BTN_D, [](){ // REGEN
    update_button_state(3, digitalRead(BTN_D));
  }, CHANGE);
  #endif
}