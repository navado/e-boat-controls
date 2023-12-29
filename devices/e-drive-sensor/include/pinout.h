#ifndef PINOUT_H
#define PINOUT_H
#include <Arduino.h>

#define LED_BRD PB5         // D13 LED on board
#define RPM_PIN 5           // D5/T1 -> Yellow Hall sensor wire on one of the motors
#define THROTTLE_IN A0      // 1k/100k voltage divider -> Throttle filter out for controll
#define THROTTLE_OUT 6      // D6 PWM -> Throttle RC filter, 2 stages -> Throttle in on all motors
#define VCC_SENS_IN A1      // 470k/20k voltage divider 453k/20k measured

#define RELAY_LOCK PB0      //D8 -> Relay 1
#define RELAY_REVERSE PD7   //D7 -> Relay 2
#define RELAY_REGEN PD4     //D4 -> Relay 3
#define RELAY_SPARE PD2     //D2 -> Relay 4

#define CS_PIN 12
#define RST_PIN 11
#define RS_PIN 10
#define SCL_PIN A6
#define SDO_PIN A7
#define BACKLIGHT_PIN 9     // D9 PWM Not working on current boards. Using digitalWrite instead

#define BTN_D A5
#define BTN_C A4
#define BTN_B A3
#define BTN_A A2

#endif