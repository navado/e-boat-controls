#include <Arduino.h>
#include <U8g2lib.h>


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/* ------------ PINOUT ---------------*/
#pragma region PINOUT


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

#define BTN_PRESS_THRESHOLD 2
#define BTN_LONG_PRESS_THRESHOLD 40
#define BTN_MAX_DURATION 120

#define BTN_MILLS ((millis()/100) & 0x003F)
#define BTN_PRESS(btn)  (btn->state == 1 && BTN_MILLS - btn->t > BTN_PRESS_THRESHOLD)
#define BTN_LONG_PRESS(btn)  (btn->state == 1 && BTN_MILLS - btn->t > BTN_LONG_PRESS_THRESHOLD)

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


#define THROTTLE_TABLE_SIZE 11
const uint8_t throttle_table[] = {245, 224, 185, 145, 110, 1, 110, 145, 185, 224, 254};
#define SPD_NEUTRAL 5


volatile uint16_t throttle_in_value = 0;

typedef struct
{
  union{
    struct {
      uint8_t state : 1; // 0 - released, 1 - pressed
      uint8_t ov: 1;
      uint8_t t: 6;
    };
    uint8_t input;
  };
  union{
  struct{
    uint8_t short_press : 1;
    uint8_t long_press : 1;
    uint8_t very_long_press : 1;
    uint8_t ht : 6;
  };
  uint8_t handle;
  };
} btn_state_t;


btn_state_t buttons_state[4] = {0};
btn_state_t * btn_a = &buttons_state[0];
btn_state_t * btn_b = &buttons_state[1];
btn_state_t * btn_c = &buttons_state[2];
btn_state_t * btn_d = &buttons_state[3];


#include <models.h>
#pragma endregion

/* ------------ Functions ------------------*/

void update_button_state(uint8_t index, uint8_t value){
  btn_state_t * btn = &buttons_state[index];

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

void update_buttons(){
  update_button_state(0, digitalRead(BTN_A));
  update_button_state(1, digitalRead(BTN_B));
  update_button_state(2, digitalRead(BTN_C));
  update_button_state(3, digitalRead(BTN_D));
}

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


  // Buttons
  pinMode(BTN_A, INPUT);
  pinMode(BTN_B, INPUT);
  pinMode(BTN_C, INPUT);
  pinMode(BTN_D, INPUT);


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
  panel_state.speed = SPD_NEUTRAL;
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
  _y = printKWLabel(_x, _y, "T: ", millis()/1000);
  _y = printKWLabel(_x, _y, "PWR: ", btn_c->t);
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
  lcd.setFont(u8g2_font_inb30_mr);
  lcd.setCursor(0, 63);
  if(panel_state.mode == 0){
    lcd.print("OFF");
  } else if (panel_state.regen == 1){
    lcd.print("REGEN");
  } else {
    lcd.print(panel_state.speed==SPD_NEUTRAL?"N":(panel_state.speed < SPD_NEUTRAL?"R":"F"));
    // lcd.print(panel_state.rpm);
    lcd.print(9999);
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
  analogWrite(THROTTLE_OUT, throttle_table[panel_state.speed]);
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
  // digitalWrite(LED_BRD_GR, !digitalRead(LED_BRD_GR));
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  handle_throttle();
  handle_state();
  draw_screen();
}