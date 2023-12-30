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
  update_button_state(0, digitalRead(BTN_A));
  update_button_state(1, digitalRead(BTN_B));
  update_button_state(2, digitalRead(BTN_C));
  update_button_state(3, digitalRead(BTN_D));
}

void setup_buttons(){
  // Buttons
  pinMode(BTN_A, INPUT);
  pinMode(BTN_B, INPUT);
  pinMode(BTN_C, INPUT);
  pinMode(BTN_D, INPUT);

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