# Controls for DYI electric boat

[![CI](https://github.com/navado/e-boat-pannel/actions/workflows/ci.yml/badge.svg)](https://github.com/navado/e-boat-pannel/actions/workflows/ci.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

Firmware for a two-board electric-boat motor control system. A **sensor node** (Arduino Nano) reads hall-effect RPM pulses, a throttle potentiometer, and battery voltage, then drives four relays and a PWM throttle signal. A **panel node** (Arduino Nano or STM32 Bluepill) provides an SPI-connected 128×64 LCD display and four tactile buttons for the operator. The two nodes communicate over a serial link using a checksummed NMEA-0183-style protocol.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Hardware Bill of Materials](#hardware-bill-of-materials)
3. [Wiring Diagrams](#wiring-diagrams)
   - [Sensor Node (Arduino Nano)](#sensor-node--arduino-nano)
   - [Panel Node — STM32 Bluepill](#panel-node--stm32-bluepill)
   - [Panel Node — Arduino Nano (alternative)](#panel-node--arduino-nano-alternative)
   - [System interconnect](#system-interconnect)
4. [Communication Protocol](#communication-protocol)
5. [Throttle Table](#throttle-table)
6. [Safety Interlocks](#safety-interlocks)
7. [Building the Firmware](#building-the-firmware)
8. [Running the Tests](#running-the-tests)
9. [Project Layout](#project-layout)
10. [License](#license)

---

## System Architecture

```
  ┌─────────────────────────────┐        ┌──────────────────────────────────┐
  │       PANEL NODE            │        │          SENSOR NODE             │
  │  (STM32 Bluepill or Nano)   │        │         (Arduino Nano)           │
  │                             │  UART  │                                  │
  │  [BTN_A] Throttle UP   ─┐  │◄──────►│  ┌─ Relay: main power lock       │
  │  [BTN_B] Throttle DOWN ─┤  │        │  ├─ Relay: reverse direction      │
  │  [BTN_C] Power toggle  ─┤  │        │  ├─ Relay: regenerative braking   │
  │  [BTN_D] Regen toggle  ─┘  │        │  └─ Relay: spare                 │
  │                             │        │                                  │
  │  ┌─────────────────┐        │        │  PWM ──► Motor controller        │
  │  │  ST7565 128×64  │        │        │                                  │
  │  │    LCD display  │        │        │  Hall sensor ──► RPM counter     │
  │  └─────────────────┘        │        │  Throttle pot ──► ADC           │
  └─────────────────────────────┘        │  48 V bus ──► Voltage divider   │
                                         └──────────────────────────────────┘
```

### Serial link messages

| Direction | Message | Trigger |
|-----------|---------|---------|
| Panel → Sensor | `$ENCMD,pow:on/off,rev:on/off,reg:on/off,thr:<0-255>*HH` | Any button press |
| Sensor → Panel | `$ENINF,<T>,<rpm>,<pow>,<rev>,<reg>,<thr>,<vthr>,<vcc>,0*HH` | Every 10 s |

---

## Hardware Bill of Materials

| Component | Qty | Notes |
|-----------|-----|-------|
| Arduino Nano (ATmega328P) | 2 | One for sensor, one for Nano-panel variant |
| STM32 Bluepill (F103C8) | 1 | Alternative panel; J-Link or ST-Link required for first flash |
| ST7565-based 128×64 LCD | 1 | ERC12864 or compatible, 4-wire SPI |
| Tactile push-buttons | 4 | Normally open, 10 kΩ pull-down resistors |
| Hall-effect sensor | 1 | Open-drain NPN; 1 kΩ pull-up to 5 V |
| N-channel MOSFETs / relay board | 4 | 5 V coil, rated for motor current |
| Resistors: 1 kΩ, 100 kΩ | 1 each | Throttle pot voltage divider |
| Resistors: 470 kΩ, 20 kΩ | 1 each | 48 V battery voltage divider |
| RC low-pass filter (1 kΩ + 10 µF) | 1 | Smooths PWM throttle signal |
| 48 V lithium battery pack | 1 | Nominal voltage; confirm relay ratings |

---

## Wiring Diagrams

### Sensor Node — Arduino Nano

```
Arduino Nano
─────────────────────────────────────────────────────────────
Pin  Label   Direction  Connected to
─────────────────────────────────────────────────────────────
D0   RX       IN         Panel UART TX  (3.3 V logic — use level shifter for STM32)
D1   TX       OUT        Panel UART RX
D2   RELAY_SPARE  OUT   Relay board IN4 (spare)
D4   RELAY_REGEN  OUT   Relay board IN3 (regenerative braking enable)
D5   T1/RPM   IN         Hall sensor OUT  ──[1 kΩ pull-up to 5 V]── VCC
                          (falling-edge counted by Timer 1; 6 pulses/rev)
D6   THROTTLE_OUT OUT   RC filter ──[1 kΩ]──┬── Motor controller throttle IN
                                             └──[10 µF]── GND
D7   RELAY_REV    OUT   Relay board IN2 (motor direction reversal)
D8   RELAY_LOCK   OUT   Relay board IN1 (main power lock)
D13  LED_BRD  OUT        On-board LED (heartbeat blink)
A0   THROTTLE_IN  IN    Throttle feedback divider:
                          48 V bus ──[470 kΩ]──┬── A0
                                               └──[20 kΩ]── GND
A1   VCC_SENS_IN  IN    Battery voltage divider:
                          5 V pot wiper ──[1 kΩ]──┬── A1
                                                  └──[100 kΩ]── GND
A2   BTN_A    IN         Button A (Throttle UP)  ──[10 kΩ pull-down]── GND
A3   BTN_B    IN         Button B (Throttle DOWN)──[10 kΩ pull-down]── GND
A4   BTN_C    IN         Button C (Power toggle) ──[10 kΩ pull-down]── GND
A5   BTN_D    IN         Button D (Regen toggle) ──[10 kΩ pull-down]── GND
─────────────────────────────────────────────────────────────

Relay board wiring (each relay independently):
  IN_n ──── Arduino output pin (active HIGH)
  COM  ──── 48 V bus or motor lead
  NO   ──── load (motor lead / power bus)
  NC   ──── leave disconnected (or use for safety tie-down)

Hall sensor wiring:
  VCC  ──── 5 V
  GND  ──── GND
  OUT  ──[1 kΩ pull-up]── 5 V
        └──── D5 (T1)
```

---

### Panel Node — STM32 Bluepill

```
STM32 Bluepill (F103C8)
─────────────────────────────────────────────────────────────
Pin   Label          Direction  Connected to
─────────────────────────────────────────────────────────────
A9    USART1 TX      OUT        Sensor RX  (use 3.3 V→5 V level shifter)
A10   USART1 RX      IN         Sensor TX  (5 V→3.3 V level shifter or 10 kΩ divider)
A0    RS_PIN         OUT        LCD RS  (register select / data-cmd)
A1    SCL_PIN        OUT        LCD SCL (serial clock)
A2    SDO_PIN        OUT        LCD SDO (serial data, MOSI)
A3    BACKLIGHT_PIN  OUT        LCD backlight anode  ──[100 Ω]── LED
B0    BTN_D          IN         Button D (Regen toggle) ──[10 kΩ pull-down]── GND
B1    BTN_C          IN         Button C (Power toggle) ──[10 kΩ pull-down]── GND
B10   BTN_B          IN         Button B (Throttle DOWN)──[10 kΩ pull-down]── GND
B11   BTN_A          IN         Button A (Throttle UP)  ──[10 kΩ pull-down]── GND
PC13  LED_BRD        OUT        On-board LED (heartbeat, active LOW on Bluepill)
PC14  CS_PIN         OUT        LCD CS  (chip select, active LOW)
PC15  RST_PIN        OUT        LCD RST (reset, active LOW)
3.3   VCC            —          LCD VCC (3.3 V)
GND   GND            —          LCD GND, button common
─────────────────────────────────────────────────────────────

LCD ERC12864 (ST7565) SPI wiring:
  VDD ──── 3.3 V
  GND ──── GND
  CS  ──── PC14
  RST ──── PC15
  RS  ──── A0
  SCL ──── A1
  SDO ──── A2
  BLA ──[100 Ω]──── A3 (PWM backlight)
  BLK ──── GND

J-Link / ST-Link programming header (SWD):
  SWDIO ──── PA13
  SWCLK ──── PA14
  GND   ──── GND
  3.3 V ──── 3.3 V  (do NOT power from programmer if board is externally powered)
─────────────────────────────────────────────────────────────
```

---

### Panel Node — Arduino Nano (alternative)

```
Arduino Nano (panel variant, compile with -D PANNEL_NANO)
─────────────────────────────────────────────────────────────
Pin   Label          Direction  Connected to
─────────────────────────────────────────────────────────────
D0    RX             IN         Sensor TX
D1    TX             OUT        Sensor RX
D2    SCL_PIN        OUT        LCD SCL
D3    SDO_PIN        OUT        LCD SDO (MOSI)
D4    CS_PIN         OUT        LCD CS  (active LOW)
D5    RST_PIN        OUT        LCD RST (active LOW)
D6    BACKLIGHT_PIN  OUT        LCD backlight ──[100 Ω]── LED
D7    RS_PIN         OUT        LCD RS  (register select)
D9    BTN_D          IN         Button D ──[10 kΩ pull-down]── GND
D10   BTN_C          IN         Button C ──[10 kΩ pull-down]── GND
D11   BTN_B          IN         Button B ──[10 kΩ pull-down]── GND
D12   BTN_A          IN         Button A ──[10 kΩ pull-down]── GND
D13   LED_BRD        OUT        On-board LED (heartbeat)
5 V   VCC            —          LCD VCC (check LCD voltage spec; add 3.3 V LDO if needed)
GND   GND            —          LCD GND, button common
─────────────────────────────────────────────────────────────
```

---

### System Interconnect

```
  ┌─────────────────────────┐              ┌─────────────────────────┐
  │      PANEL NODE         │              │       SENSOR NODE       │
  │  (STM32 or Nano)        │    UART      │     (Arduino Nano)      │
  │                         │  115200 8N1  │                         │
  │  USART TX ──────────────┼─────────────►│ RX (D0)                 │
  │  USART RX ◄─────────────┼──────────────┤ TX (D1)                 │
  │                         │              │                         │
  │  5 V ───────────────────┼──────────────┤ 5 V  (shared supply)    │
  │  GND ───────────────────┼──────────────┤ GND                     │
  └─────────────────────────┘              └─────────────────────────┘

  Note: STM32 Bluepill I/O is 3.3 V. When connecting to a 5 V Nano sensor:
    TX (STM32 → Nano RX): direct connection — Nano RX tolerates 3.3 V.
    RX (Nano TX → STM32): use a 10 kΩ / 20 kΩ voltage divider or level shifter.
```

---

## Communication Protocol

Messages use an NMEA-0183-inspired framing:

```
$<PAYLOAD>*<XX>\n
 │              │└─ Newline (0x0A)
 │              └── Two hex digits: XOR checksum of all bytes in PAYLOAD
 └── PAYLOAD: comma-separated fields
```

### Panel → Sensor: `ENCMD`

```
$ENCMD,pow:<on|off>,rev:<on|off>,reg:<on|off>,thr:<0-255>*XX
```

| Field | Values | Meaning |
|-------|--------|---------|
| `pow` | `on` / `off` | Main power relay |
| `rev` | `on` / `off` | Reverse relay |
| `reg` | `on` / `off` | Regenerative-braking relay |
| `thr` | 0–255 | PWM throttle value from throttle table |

### Sensor → Panel: `ENINF`

```
$ENINF,<T>,<rpm>,<pow>,<rev>,<reg>,<thr>,<vthr>,<vcc>,0*XX
```

| Field | Type | Meaning |
|-------|------|---------|
| `T` | ms | Uptime timestamp |
| `rpm` | uint | Motor RPM (TCNT1 / 6) |
| `pow` | 0/1 | Power relay state |
| `rev` | 0/1 | Reverse relay state |
| `reg` | 0/1 | Regen relay state |
| `thr` | 0–255 | Active throttle PWM value |
| `vthr` | 0–5000 | Throttle ADC mapped 0–5000 mV |
| `vcc` | raw | Battery voltage ADC reading |

### Timing

| Event | Period |
|-------|--------|
| Button scan | 100 ms |
| Throttle / voltage ADC read | 100 ms |
| RPM counter latch | 1 s |
| `ENINF` telemetry transmit | 10 s |
| `ENCMD` command transmit | On change |

---

## Throttle Table

Speed index 5 is neutral. Below 5 is reverse, above 5 is forward. The PWM values are intentionally inverted (lower PWM = more current in one direction, higher = more in the other) to match the motor controller's input range.

```
Index │  0    1    2    3    4  │  5  │  6    7    8    9   10
──────┼─────────────────────────┼─────┼──────────────────────────
PWM   │ 245  224  185  145  110 │  1  │ 110  145  185  224  254
Mode  │◄────── REVERSE ─────────┤STOP │──────── FORWARD ────────►
```

Long-press on throttle buttons jumps ±4 steps; short-press moves ±1 step.

---

## Safety Interlocks

The sensor firmware enforces the following guards against unsafe state transitions:

| Command | Blocked when |
|---------|-------------|
| Power OFF | Throttle PWM > 1 (motor still spinning) |
| Reverse toggle | Throttle > 1 **or** regen is active |
| Regen toggle | Throttle > 1 **or** reverse is active |
| Throttle change | Power is off **or** regen is active |

The panel firmware adds a second layer:

| Button action | Blocked when |
|--------------|-------------|
| Power toggle (BTN_C) | Speed ≠ neutral **or** regen is active |
| Regen toggle (BTN_D) | Power is off **or** speed ≠ neutral |
| Throttle change | Power is off **or** regen is active |

---

## Building the Firmware

### Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) (CLI or IDE plugin)
- For STM32 target: J-Link or ST-Link debugger

```bash
cd devices/e-drive-sensor

# Sensor node (Arduino Nano)
pio run -e sensor

# Panel node — STM32 Bluepill
pio run -e stm-panel

# Panel node — Arduino Nano
pio run -e avr-pannel
```

### Uploading

```bash
# Connect Arduino Nano via USB, then:
pio run -e sensor --target upload
pio run -e avr-pannel --target upload

# STM32 Bluepill via J-Link:
pio run -e stm-panel --target upload
```

---

## Running the Tests

The test suite targets pure logic (tokenizer, checksum, command parser, throttle model) and requires only a C++14 compiler — no embedded hardware or PlatformIO native platform download needed.

```bash
cd devices/e-drive-sensor

g++ -std=c++14 -I include -I test \
    -D NATIVE_TEST -D LOG_LEVEL=4 \
    src/comms.cpp src/models.cpp test/test_main.cpp \
    -o /tmp/e_boat_tests && /tmp/e_boat_tests
```

Expected output:

```
PASS: test_tokenize_basic
PASS: test_tokenize_single_token
...
-----------------------
37 Tests  0 Failures
OK
```

Alternatively, once PlatformIO's native platform is downloaded:

```bash
pio test -e native
```

---

## Project Layout

```
e-boat-pannel/
└── devices/
    └── e-drive-sensor/         # Unified firmware project
        ├── platformio.ini      # Four build environments
        ├── include/
        │   ├── pinout.h        # Pin assignments for all board variants
        │   ├── models.h        # State structs, throttle table constants
        │   ├── comms.h         # Protocol types, function declarations
        │   ├── buttons.h       # Button state machine types & macros
        │   └── ui.h            # LCD display function declarations
        ├── src/
        │   ├── main-sensor.cpp # Sensor node entry point & business logic
        │   ├── main-pannel.cpp # Panel node entry point & UI logic
        │   ├── comms.cpp       # Serial framing, tokenizer, checksum
        │   ├── models.cpp      # State initialisation, throttle table
        │   ├── buttons.cpp     # Button debounce & interrupt setup
        │   └── ui.cpp          # u8g2-based LCD rendering
        └── test/
            ├── test_main.cpp   # 37 unit tests (Unity framework)
            ├── unity.h         # Bundled minimal Unity test runner
            ├── Arduino.h       # Shim: redirects to mock_arduino.h
            └── mock_arduino.h  # Arduino API stub for native compilation
```

---

## License

GNU General Public License v3.0 — see [LICENSE](LICENSE).
