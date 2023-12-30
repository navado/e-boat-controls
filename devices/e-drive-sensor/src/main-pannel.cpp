#include <Arduino.h>
#include <ui.h>
#include <pinout.h>
#include <models.h>
#include <buttons.h>

void setup() {
  Serial.begin(115200);
  // Prepare keepalive LED
  pinMode(LED_BRD, OUTPUT);
  digitalWrite(LED_BRD, HIGH);
  setup_buttons();
  setup_screen();
}


void handle_throttle(){

  if (btn_a->state == btn_b->state)
    return;

  if (panel_state.power == 0 || panel_state.regen == 1)
    return;

  if (BTN_PRESS(btn_a) && btn_a->short_press == 0){
    btn_a->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.speed = min(panel_state.speed + 1, THROTTLE_TABLE_SIZE - 1);
  }

  if(BTN_LONG_PRESS(btn_a) && btn_a->long_press == 0){
    btn_a->long_press = 1;
    btn_a->ht = BTN_MILLS;
    if(panel_state.speed > SPD_NEUTRAL) { // Forward to faster
      panel_state.speed = min(panel_state.speed + 4, THROTTLE_TABLE_SIZE - 1);
    } else if (panel_state.speed == SPD_NEUTRAL){ // Neutral to Forward
      panel_state.speed = SPD_NEUTRAL;
    } else { // Reverse to Neutral
      panel_state.speed = SPD_NEUTRAL;
    }
  }

  if (BTN_PRESS(btn_b) && btn_b->short_press == 0){
    btn_b->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.speed = max(panel_state.speed - 1, 0);
  }

  if (BTN_LONG_PRESS(btn_b) && btn_b->long_press == 0){
    btn_b->long_press = 1;
    btn_a->ht = BTN_MILLS;
    if(panel_state.speed > SPD_NEUTRAL) { // Forward to Neutral
      panel_state.speed = max(panel_state.speed - 4, SPD_NEUTRAL);
    } else if (panel_state.speed == SPD_NEUTRAL){ // Neutral to Reverse
      panel_state.speed = SPD_NEUTRAL;
    } else { // Reverse to faster
      panel_state.speed = 0;
    }
  }
}

void handle_state(){
  if (btn_c->state==btn_d->state) return;
  
  if (BTN_PRESS(btn_c) && btn_c->short_press ==0 && panel_state.regen == 0 && panel_state.speed == SPD_NEUTRAL){
    btn_c->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.power = !panel_state.power;
    if (panel_state.power == 0){
      panel_state.speed = SPD_NEUTRAL;
    }
  }
  if (BTN_PRESS(btn_d) && btn_d->short_press ==0 && panel_state.power == 1 && panel_state.speed == SPD_NEUTRAL){
    btn_d->short_press = 1;
    btn_a->ht = BTN_MILLS;
    panel_state.regen = !panel_state.regen;
  }
}

void loop()
{
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  handle_throttle();
  handle_state();
  draw_screen();
}