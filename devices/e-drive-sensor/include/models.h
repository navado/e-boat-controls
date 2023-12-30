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
  uint8_t speed: 6; // 0 - 11 index in throttle_table
  uint16_t rpm :16;
} panel_state_t;

extern volatile panel_state_t panel_state;

typedef struct {
  uint8_t power:1;          // 0 - OFF, 1 - ON
  uint8_t reverse:1;        // 0 - OFF, 1 - ON
  uint8_t regen: 1;         // 0 - OFF, 1 - ON
  uint8_t spare: 6;
  uint8_t throttle: 8;      // For analogWrite
  uint16_t throttle_val;    // As read from ADC
  uint16_t vcc48v;          // Batery voltage
  uint16_t rpm;             // RPM
  unsigned long T;          // Timestamp
} engine_state_t;
extern volatile engine_state_t engine_state;

typedef enum {
  off = 0,
  on = 1,
  error = 2
} on_off_t;
on_off_t parse_on_off(String val);
#define UPDATE_ON_OFF_FIELD(f,v) if(v!=on_off_t::error) engine_state.f = v;


#define THROTTLE_TABLE_SIZE 11
extern const uint8_t throttle_table[];

#define SPD_NEUTRAL 5

void update_throttle_value(int val);
#endif