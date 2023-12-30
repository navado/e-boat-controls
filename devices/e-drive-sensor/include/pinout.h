#ifndef PINOUT_H
#define PINOUT_H
#include <Arduino.h>
#if defined(SENSOR)
// Arduino Nano based board
#define LED_BRD 13          // D13 LED on board
#define RPM_PIN 5           // D5/T1 -> Yellow Hall sensor wire on one of the motors
#define THROTTLE_IN A0      // 1k/100k voltage divider -> Throttle filter out for controll
#define THROTTLE_OUT 6      // D6 PWM -> Throttle RC filter, 2 stages -> Throttle in on all motors
#define VCC_SENS_IN A1      // 470k/20k voltage divider 453k/20k measured

#define RELAY_LOCK 8         //D8 -> Relay 1
#define RELAY_REVERSE 7      //D7 -> Relay 2
#define RELAY_REGEN 4        //D4 -> Relay 3
#define RELAY_SPARE 2        //D2 -> Relay 4

#define BTN_D A5
#define BTN_C A4
#define BTN_B A3
#define BTN_A A2
#elif defined(PANNEL_STM32)
// Bluepill based board
#define LED_BRD PC_13          // C13 LED on board
#define CS_PIN PC_14
#define RST_PIN PC_15
#define RS_PIN A0
#define SCL_PIN A1
#define SDO_PIN A2
#define BACKLIGHT_PIN A3     // D9 PWM Not working on current boards. Using digitalWrite instead

#define BTN_D B0
#define BTN_C B1
#define BTN_B B10
#define BTN_A B11
#elif defined(PANNEL_NANO)
// Arduino Nano based board
#define LED_BRD         13
#define CS_PIN          4
#define RST_PIN         5
#define RS_PIN          7
#define SCL_PIN         2
#define SDO_PIN         3
#define BACKLIGHT_PIN   6

#define BTN_D           9
#define BTN_C           10
#define BTN_B           11
#define BTN_A           12
#endif
#endif