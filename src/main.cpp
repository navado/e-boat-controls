#include <Arduino.h>
#include <U8g2lib.h>
#include "TimerInterrupt_Generic.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/* ------------ PINOUT ---------------*/
#pragma region PINOUT

#if defined(BOARD_ARDUINO_NANO)
#define CS_PIN 8
#define RSE_PIN 7
#define RS_PIN 6
#define SCL_PIN 12 // to release 11 for PWM
#define SDO_PIN 4
#define SDI_PIN 10
#define BACKLIGHT_PIN 11 // PWM
#define LED_BRD_RD 13
#define LED_BRD_GR 10

#define RPM_PIN 5

#define THROTTLE_IN A0
#define THROTTLE_OUT 3 // PWM

#define BTN_D A2
#define BTN_C A3
#define BTN_B A4
#define BTN_A A5
#elif defined(BOARD_BLUEPILL)


#define CS_PIN PC14
#define RST_PIN PC15
#define RS_PIN PA0
#define SCL_PIN PA1
#define SDO_PIN PA2
#define SDI_PIN PB5
#define BACKLIGHT_PIN PA3 // PWM
#define LED_BRD LED_BUILTIN

#define RPM_PIN PB5 

#define THROTTLE_IN PB1  //  ADC8
#define THROTTLE_OUT PB0 // PWM 19

#define BTN_D PB6
#define BTN_C PB7
#define BTN_B PB8
#define BTN_A PB9
#endif

#define BTN_PRESS_THRESHOLD 1
#define BTN_LONG_PRESS_THRESHOLD 60
#define BTN_MAX_DURATION 120
#pragma endregion
/* ------------ LCD ------------------*/

#pragma region LCD
#define CONTRAST_SETTING 0x3A
uint8_t backlight_value = 30;

U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI lcd(U8G2_R0,
                                        /* clock=*/SCL_PIN,
                                        /* data=*/SDO_PIN,
                                        /* cs=*/CS_PIN,
                                        /* dc=*/RS_PIN,
                                        /* reset=*/RST_PIN
                                        );

#pragma endregion

/* ------------ Variables ------------------*/
#pragma region Variables
volatile uint8_t frames = 0;
volatile uint8_t fps = 0;
volatile uint16_t cnt, cnt_raw = 0, ticks = 0;


#define THROTTLE_TABLE_SIZE 11
const uint8_t throttle_table[] = {245, 224, 185, 145, 110, 1, 110, 145, 185, 224, 254};
#define SPD_NEUTRAL 5


volatile uint16_t throttle_in_value = 0;

typedef struct
{
  uint8_t state : 1; // 0 - released, 1 - pressed
  uint8_t count: 7;
} btn_state_t;

btn_state_t buttons_state[4] = {0};
btn_state_t * btn_a = &buttons_state[0];
btn_state_t * btn_b = &buttons_state[1];
btn_state_t * btn_c = &buttons_state[2];
btn_state_t * btn_d = &buttons_state[3];


volatile struct panel_state_ {
  uint8_t mode: 1; // 0 - OFF, 1 - ON
  uint8_t regen: 1; // 0 - OFF, 1 - ON
  uint8_t speed: 6; // 0 - 11 index in throttle_table
  uint16_t rpm :16;
} panel_state = {0};
#pragma endregion

/* ------------ Functions ------------------*/

void update_button_state(uint8_t index, uint8_t value){
  btn_state_t * btn = &buttons_state[index];
  btn->state = value;
  btn->count = (btn->state == 1 && btn->count<120)? btn->count + 1 : btn->count;
  if (btn->state == 0){
    btn->count = 0;
  }
}

void update_buttons(){
  update_button_state(0, digitalRead(BTN_A));
  update_button_state(1, digitalRead(BTN_B));
  update_button_state(2, digitalRead(BTN_C));
  update_button_state(3, digitalRead(BTN_D));
}

void setup_rpm_counter(){
  // pinMode(RPM_PIN, INPUT);
  #if defined(BOARD_ARDUINO_NANO)
  TCCR1A=0; // setup timer-1
  TCCR1C=0;
  TIMSK1=0;
  GTCCR=0;
  TCCR1B=0b00000110; // falling edge
  #endif

}

void run_every_1s(){
  #if defined(BOARD_ARDUINO_NANO)
  cnt = TCNT1;
  TCNT1=0;
  #endif
  panel_state.rpm = cnt;
  cnt=0;
}

void run_every_100ms(){
  // update_buttons();
  throttle_in_value = analogRead(THROTTLE_IN);
  ticks++;
  cnt++;
  if (ticks % 10 == 0){
    run_every_1s();
  }
}

#if defined(BOARD_ARDUINO_NANO)
TimerInterrupt timer1(1);

void setup_timing_functions(){
  timer1.attachInterruptInterval(100, run_every_100ms);
}
#elif defined(BOARD_BLUEPILL)
STM32TimerInterrupt timer1(TIM1);

void setup_timing_functions(){
  timer1.attachInterruptInterval(100000, run_every_100ms);
}
#endif

void setup() {
  Serial.begin(115200);
  // Prepare keepalive LED
  pinMode(LED_BRD, OUTPUT);
  digitalWrite(LED_BRD, HIGH);

  
  // Prepare backlight
  pinMode(BACKLIGHT_PIN, OUTPUT);
  analogWrite(BACKLIGHT_PIN, backlight_value);
  
  lcd.begin();
  lcd.setFont(u8g2_font_5x8_tf);

  lcd.setContrast(CONTRAST_SETTING);
  Serial.println("LCD Init Done");

  // Prepare timings
  setup_timing_functions();

  // Throttle
  pinMode(THROTTLE_IN, INPUT);
  pinMode(THROTTLE_OUT, OUTPUT);
  analogWrite(THROTTLE_OUT, throttle_table[panel_state.speed]);

  // Buttons
  pinMode(BTN_A, INPUT);
  pinMode(BTN_B, INPUT);
  pinMode(BTN_C, INPUT);
  pinMode(BTN_D, INPUT);

  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(RPM_PIN, [](){
    cnt++;
  }, CHANGE);
  attachInterrupt(BTN_A, [](){ // Throttle UP
    update_button_state(0, digitalRead(BTN_A));
  }, RISING);
  attachInterrupt(BTN_B, [](){ // Throttle DOWN
    update_button_state(1, digitalRead(BTN_B));
  }, RISING);
  attachInterrupt(BTN_C, [](){ // Reverse
    update_button_state(2, digitalRead(BTN_C));
  }, RISING);
  attachInterrupt(BTN_D, [](){ // REGEN
    update_button_state(3, digitalRead(BTN_D));
  }, RISING);
  panel_state.speed = SPD_NEUTRAL;
  // Timers/counters
  setup_rpm_counter();
}

#pragma region UI
void printSmiley(uint8_t x, uint8_t y, uint8_t r){
  uint8_t _eye_level = y - r/4;
  uint8_t _eye_size = r/8;
  uint8_t _eye_spacing = r/3;
  uint8_t _mouth_size = 2*r/3;
  lcd.drawCircle(x,y, r); // body
  lcd.drawArc(x, y, _mouth_size, 137, 244); // mouth
  lcd.drawDisc(x-_eye_spacing, _eye_level , _eye_size);  // left eye
  lcd.drawDisc(x+_eye_spacing, _eye_level, _eye_size);  // right eye
}

uint8_t printKWLabel(uint8_t x, uint8_t y, const char* key,  uint32_t value){
  lcd.setFont(u8g2_font_5x8_tf);
  lcd.setCursor(x, y);
  lcd.print(key);
  lcd.print(value);
  return y+8;
}

uint8_t printKWLabel(uint8_t x, uint8_t y, const char* key, const char * value){
  lcd.setFont(u8g2_font_5x8_tf);
  lcd.setCursor(x, y);
  lcd.print(key);
  lcd.print(value);
  return y+8;
}

void printColumn1(){
  uint8_t _x=2, _y = 8;
  _y = printKWLabel(_x, _y, "CNT: ", cnt);
}


void printColumn2(){
  uint8_t _x=64, _y = 8;
  _y = printKWLabel(_x, _y, "T/I: ", panel_state.speed);
  _y = printKWLabel(_x, _y, "T/O: ", throttle_table[panel_state.speed]);
  _y = printKWLabel(_x, _y, "T/V: ", map(throttle_in_value, 0, 1023, 0, 500));
}

void printSmileyFont(uint8_t index){
  lcd.setFont(u8g2_font_emoticons21_tr);
  lcd.setCursor(2, 60);
  lcd.print(index);
}

void printBigLabel(){
  lcd.setFont(u8g2_font_inb21_mr);
  lcd.setCursor(2, 63);
  if(panel_state.mode == 0){
    lcd.print("OFF");
  } else {
    lcd.print(panel_state.speed==SPD_NEUTRAL?"N":(panel_state.speed < SPD_NEUTRAL?"R":"F"));
    lcd.print(panel_state.rpm);
  }
}

void draw_screen(){
  analogWrite(BACKLIGHT_PIN, backlight_value);
  frames++;
  lcd.firstPage();
  do
  {
    printColumn1();
    printColumn2();
    printBigLabel();
  } while (lcd.nextPage());
}
#pragma endregion

// typedef enum{
//   RELEASED = 0,
//   PRESSED = 1,
//   LONG_PRESSED = 2,
//   MAX_DURATION = 3,
//   OTHER = 4
// } button_state_t;

// button_state_t get_button_state(uint8_t index){
//   btn_state_t * btn = &buttons_state[index];
//   if (btn->state == 0)
//     return RELEASED;
//   if (btn->count == BTN_PRESS_THRESHOLD)
//     return PRESSED;
//   if (btn->count == BTN_LONG_PRESS_THRESHOLD)
//     return LONG_PRESSED;
//   if (btn->count == BTN_MAX_DURATION)
//     return MAX_DURATION;
//   return OTHER;
// }

void handle_throttle(){

  if (btn_a->state == btn_b->state)
    return;

  if (btn_a->count == BTN_PRESS_THRESHOLD){
    btn_a->count = btn_a->count + 1;
    panel_state.speed = min(panel_state.speed + 1, THROTTLE_TABLE_SIZE - 1);
  }

  if(btn_a->count == BTN_LONG_PRESS_THRESHOLD){
    btn_a->count = btn_a->count + 1;
    panel_state.speed = min(panel_state.speed + 4, THROTTLE_TABLE_SIZE - 1);
  }

  if (btn_b->count == BTN_PRESS_THRESHOLD){
    btn_b->count = btn_b->count + 1;
    panel_state.speed = max(panel_state.speed - 1, 0);
  }

  if (btn_b->count == BTN_PRESS_THRESHOLD){
    btn_b->count = btn_b->count + 1;
    panel_state.speed = max(panel_state.speed - 4, 0);
  }
  analogWrite(THROTTLE_OUT, throttle_table[panel_state.speed]);
}

void handle_state(){
  if (btn_c->state==btn_d->state) return;

  if (btn_c->count == BTN_PRESS_THRESHOLD){
    panel_state.mode = !panel_state.mode;
    if (panel_state.mode == 0){
      panel_state.speed = SPD_NEUTRAL;
    }
  }
  if (btn_d->count == BTN_PRESS_THRESHOLD ){
    panel_state.regen = !panel_state.regen;
  }
}

void loop()
{
  // digitalWrite(LED_BRD_GR, !digitalRead(LED_BRD_GR));
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  handle_throttle();
  handle_state();
  draw_screen();
}