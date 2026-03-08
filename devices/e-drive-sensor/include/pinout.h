#ifndef PINOUT_H
#define PINOUT_H
#include <Arduino.h>
#if defined(SENSOR)
// Arduino Nano based board
#define LED_BRD 13          // D13 LED on board
#define RPM_PIN 5           // D5/T1 -> Hall sensor (motor RPM, 6 pulses/rev)
#define THROTTLE_IN A0      // 1k/100k voltage divider -> Throttle filter out
#define THROTTLE_OUT 6      // D6 PWM -> Throttle RC filter -> Motor throttle in
#define VCC_SENS_IN A1      // 470k/20k voltage divider (measures 48 V bus)
#define CURR_SENS_IN A6     // Shunt amplifier output -> motor current (mA)
// NOTE: water speed comes from NMEA0183/2000 via the throttle device (THRINF message)
// D3 (formerly impeller ISR) is now available as RELAY_SPARE or future use

#define RELAY_LOCK 8         //D8 -> Relay 1 (motor enable)
#define RELAY_REVERSE 7      //D7 -> Relay 2 (direction)
#define RELAY_REGEN 4        //D4 -> Relay 3 (regen braking)
#define RELAY_SPARE 2        //D2 -> Relay 4 (spare)

#define BTN_D A5
#define BTN_C A4
#define BTN_B A3
#define BTN_A A2

// Current sensing calibration: shunt amplifier 50mV/A, ADC Vref 5V
// curr_mA = (ADC * 5000 / 1023) / 50 * 1000  →  ADC * 97.75
#define CURR_MV_PER_AMP   50
#elif defined(PANNEL_STM32)
// Bluepill based board
// Serial  (USART1 PA9/PA10) : shared bus (sensor + panel + throttle)
// Serial3 (USART3 PB10/PB11): optional NMEA 0183 GPS input (enable -D NMEA0183_PANEL)
//   NOTE: PA2/PA3 are used by LCD (SDO/BACKLIGHT), so USART3 is used instead.
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

#if defined(NMEA0183_PANEL)
// NMEA 0183 GPS input on panel STM32 (USART3 — PA2/PA3 occupied by LCD)
#define NMEA_PANEL_RX  PB11   // USART3_RX
#define NMEA_PANEL_TX  PB10   // USART3_TX (TX not needed for GPS receive only)
#endif
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

#elif defined(THROTTLE_ESP32)
// ESP32-PICO-D4 throttle controller
// Serial  (UART0 GPIO1/GPIO3)  : shared bus (sensor + panel + throttle)
// Serial2 (UART2 GPIO16/GPIO17): NMEA0183 GPS / chart-plotter input

#define LED_BRD             2    // Built-in LED

// Throttle position input
#define THROTTLE_POT_PIN    36   // ADC1_CH0 (VP, input-only; no pullup needed for pot)
#define THROTTLE_ENC_A      32   // ADC1_CH4
#define THROTTLE_ENC_B      33   // ADC1_CH5

// Mode-selector encoder
#define MODE_ENC_A          25
#define MODE_ENC_B          26
#define MODE_ENC_BTN        27

// Control buttons
// GPIO34/35 are input-only on ESP32 (no internal pullup) — wire 10 kΩ pull-ups to 3.3 V
#define BTN_ENGINE_PIN      34
#define BTN_PANEL_PIN       35

// RGB status LED (common-cathode, PWM driven via LEDC)
#define LED_R_PIN           18
#define LED_G_PIN           19
#define LED_B_PIN           23

// Buzzer (passive, digital toggle)
#define BUZZER_PIN          4

// NMEA 0183 GPS/chart-plotter port (UART2)
// RX receives GPS NMEA sentences (enable with -D NMEA0183_THROTTLE).
// TX transmits motor NMEA sentences ($IIRPM, $IIXDR) always when THROTTLE_ESP32.
#define NMEA_ESP32_RX       16   // U2_RXD
#define NMEA_ESP32_TX       17   // U2_TXD
#if !defined(NMEA0183_PANEL)
#  define NMEA0183_THROTTLE
#endif

// NMEA 2000 CAN bus (ESP32 TWAI peripheral)
// Receives:  PGN 128259 — Speed Through Water, PGN 129026 — COG & SOG Rapid
// Transmits: PGN 127488 — Engine Parameters Rapid, PGN 127508 — Battery Status
// Enable with -D NMEA2000 build flag; requires SN65HVD230 or ISO1050 transceiver
#define NMEA2000_CAN_RX     22
#define NMEA2000_CAN_TX     21

#elif defined(THROTTLE_STM32)
// BluePill STM32F103 based throttle controller
// Serial  (USART1 PA9/PA10) : shared bus (sensor + panel + throttle)
// Serial2 (USART2 PA2/PA3)  : NMEA0183 GPS / chart-plotter input

#define LED_BRD         PC_13  // Onboard LED (active low)

// Throttle position input — one of the two below is active at compile time
#define THROTTLE_POT_PIN  PA0  // Analog potentiometer (THROTTLE_POT mode)
#define THROTTLE_ENC_A    PB0  // Quadrature encoder A (THROTTLE_ENC mode)
#define THROTTLE_ENC_B    PB1  // Quadrature encoder B (THROTTLE_ENC mode)

// Mode-selector encoder (selects RPM/PWR/SOG/SOW/RNG mode)
// PB3/PB4 are JTDO/JNTRST; safe to use as GPIO with SWD upload
#define MODE_ENC_A      PB3
#define MODE_ENC_B      PB4
#define MODE_ENC_BTN    PB5   // Push-to-select (OK / calibrate centre)

// Control buttons
#define BTN_ENGINE_PIN  PB6   // Engine ON/OFF (active low, INPUT_PULLUP)
#define BTN_PANEL_PIN   PB7   // Panel ON/OFF, long-press interlock (active low)

// RGB status LED (common-cathode, PWM driven)
#define LED_R_PIN       PA1   // TIM2_CH2
#define LED_G_PIN       PA6   // TIM3_CH1
#define LED_B_PIN       PA7   // TIM3_CH2

// Buzzer (passive, driven with tone() or analogWrite)
#define BUZZER_PIN      PA8   // TIM1_CH1

// NMEA 0183 GPS/chart-plotter port (Serial2 = USART2, PA2=TX, PA3=RX)
// RX receives GPS NMEA sentences (enable with -D NMEA0183_THROTTLE).
// TX transmits motor NMEA sentences ($IIRPM, $IIXDR) always when THROTTLE_STM32.
// Default: NMEA0183_THROTTLE is on unless another node's NMEA flag is active.
#if !defined(NMEA0183_PANEL)
#  define NMEA0183_THROTTLE   // throttle reads GPS on Serial2 RX by default
#endif

// NMEA 2000 CAN bus (requires external SN65HVD230 or equivalent transceiver)
// Receives:  PGN 128259 — Speed Through Water, PGN 129026 — COG & SOG Rapid
// Transmits: PGN 127488 — Engine Parameters Rapid, PGN 127508 — Battery Status
// Enable with -D NMEA2000 build flag
#define NMEA2000_CAN_RX  PA11  // CAN_RX (remapped)
#define NMEA2000_CAN_TX  PA12  // CAN_TX (remapped)

#endif
#endif