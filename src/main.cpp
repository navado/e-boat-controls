#include <Arduino.h>
#include <U8g2lib.h>
#include <TimeInterrupt.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/* ------------ PINOUT ---------------*/
#pragma region PINOUT

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

#define NUM_PWM_PINS 5
uint8_t pwm_pin_test_intex = 0;
uint8_t pwm_pins_to_test[] = {3, 6, 9, 10, 11};

#define THROTTLE_IN A0
#define THROTTLE_OUT 3 // PWM

#define BTN_D A2
#define BTN_C A3
#define BTN_B A4
#define BTN_A A5
#define BTN_PRESS_THRESHOLD 4
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
                                        /* reset=*/RSE_PIN
                                        );

#pragma endregion

/* ------------ Variables ------------------*/
#pragma region Variables
volatile uint8_t frames = 0;
volatile uint8_t fps = 0;
volatile uint16_t cnt, cnt_raw = 0;


#define THROTTLE_TABLE_SIZE 12
const uint8_t throttle_table[] = {1, 80, 90, 110, 125, 145, 165, 185, 205, 224, 244, 254};

volatile int8_t throttle_index = 0; // to handle decreases
volatile uint8_t throttle_out_value = 0;
#define THROTTLE_IN_STEP (499/1024.0)
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

void setup_timer1_counter(){
  pinMode(RPM_PIN, INPUT);
  
  TCCR1A=0; // setup timer-1
  TCCR1C=0;
  TIMSK1=0;
  GTCCR=0;
  TCCR1B=0b00000110; // falling edge
}

void setup() {
  Serial.begin(115200);
  // Prepare keepalive LED
  pinMode(LED_BRD_RD, OUTPUT);
  digitalWrite(LED_BRD_RD, HIGH);
  pinMode(LED_BRD_GR, OUTPUT);
  digitalWrite(LED_BRD_GR, LOW);
  
  // Prepare backlight
  
  analogWrite(BACKLIGHT_PIN, backlight_value);
  
  lcd.begin();
  lcd.setFont(u8g2_font_5x8_tf);

  lcd.setContrast(CONTRAST_SETTING);
  Serial.println("LCD Init Done");

  // Prepare timings
  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt([]()
                             {
                              cnt = TCNT1;
                              TCNT1=0;
                              fps = frames;
                              frames = 0;
                              },
                             1000); // 1s
  TimeInterrupt.addInterrupt([]()
                             {
                              throttle_in_value = analogRead(THROTTLE_IN); 
                              update_buttons();
                             },
                             100); // 100ms


  // Throttle
  pinMode(THROTTLE_IN, INPUT);
  pinMode(THROTTLE_OUT, OUTPUT);
  analogReference(DEFAULT);

  // Buttons
  pinMode(BTN_A, INPUT);
  pinMode(BTN_B, INPUT);
  pinMode(BTN_C, INPUT);
  pinMode(BTN_D, INPUT);

  // Timers/counters
  // setup_timer1_counter();

  // iterate over all pins and send to serial theit timer mapping for analogWrite
  for (uint8_t i = 0; i < NUM_PWM_PINS; i++){
    uint8_t pin = pwm_pins_to_test[i];
    Serial.print("Pin: ");
    Serial.print(pin);
    Serial.print(" Timer: ");
    Serial.println(digitalPinToTimer(pin));
    analogWrite(pin, 50);
  }
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
  _y = printKWLabel(_x, _y, "RPM: ", cnt * 10);
  _y = printKWLabel(_x, _y, "T: ", millis()/1000);
  _y = printKWLabel(_x, _y, "FPS: ", fps);
}

const char * btn_to_str_value(uint8_t btn){
  switch (btn)
  {
  case 0:
    return "OFF";
  case 1:
    return "ON";
  default:
    return "ERR";
  }
}

uint8_t print_btn_state(uint8_t _x, uint8_t _y, uint8_t index){
  btn_state_t * btn = &buttons_state[index];
  lcd.setFont(u8g2_font_5x8_tf);
  lcd.setCursor(_x, _y);
  lcd.print("B-");
  lcd.print((char)(index + 'A'));
  lcd.print(": ");
  lcd.print(btn_to_str_value(btn->state));
  lcd.print(" ");
  lcd.print(btn->count);
  return _y+8;
}

void printColumn2(){
  uint8_t _x=64, _y = 8;
  _y = printKWLabel(_x, _y, "T/I: ", throttle_index);
  _y = printKWLabel(_x, _y, "T/O: ", throttle_out_value);
  _y = printKWLabel(_x, _y, "T/V: ", map(throttle_in_value, 0, 1023, 0, 500));
  for (uint8_t i = 0;i<4;i++){
    _y = print_btn_state(_x,_y, i);
  }
}

void printSmileyFont(uint8_t index){
  lcd.setFont(u8g2_font_emoticons21_tr);
  lcd.setCursor(2, 60);
  lcd.print(index);
}

void draw_screen(){
  analogWrite(BACKLIGHT_PIN, backlight_value);
  frames++;
  lcd.firstPage();
  do
  {
    lcd.drawFrame(0, 0, 127, 63);
    printColumn1();
    printColumn2();
    // printSmiley(62+30,31,30);
    printSmileyFont(frames%10);

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
  analogWrite(THROTTLE_OUT, throttle_out_value);
  if (btn_a->state == btn_b->state)
    return;

  if (btn_a->count == BTN_PRESS_THRESHOLD){
    btn_a->count = btn_a->count + 1;
    throttle_index = min(throttle_index + 1, THROTTLE_TABLE_SIZE - 1);
  }

  if(btn_a->count == BTN_LONG_PRESS_THRESHOLD){
    btn_a->count = btn_a->count + 1;
    throttle_index = min(throttle_index + 4, THROTTLE_TABLE_SIZE - 1);
  }

  if (btn_b->count == BTN_PRESS_THRESHOLD){
    btn_b->count = btn_b->count + 1;
    throttle_index = max(throttle_index - 1, 0);
  }

  if (btn_b->count == BTN_PRESS_THRESHOLD){
    btn_b->count = btn_b->count + 1;
    throttle_index = max(throttle_index - 4, 0);
  }


  throttle_out_value = throttle_table[throttle_index];
}

void loop()
{
  // digitalWrite(LED_BRD_GR, !digitalRead(LED_BRD_GR));
  digitalWrite(LED_BRD_RD, !digitalRead(LED_BRD_RD));
  handle_throttle();
  draw_screen();
}