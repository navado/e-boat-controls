#include <Arduino.h>
#include <U8g2lib.h>
#include <ui.h>
#include <pinout.h>
#include <models.h>
#include <buttons.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define CONTRAST_SETTING 0x3A
uint8_t backlight_value = 255;
uint8_t frames = 0;

// U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI lcd(U8G2_R0,
//                                         /* clock=*/SCL_PIN,
//                                         /* data=*/SDO_PIN,
//                                         /* cs=*/CS_PIN,
//                                         /* dc=*/RS_PIN,
//                                         /* reset=*/RST_PIN
//                                         );
U8G2_ST7565_ERC12864_ALT_2_4W_SW_SPI  lcd(U8G2_R0,
                                        /* clock=*/SCL_PIN,
                                        /* data=*/SDO_PIN,
                                        /* cs=*/CS_PIN,
                                        /* dc=*/RS_PIN,
                                        /* reset=*/RST_PIN
                                        );

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
  _y = printKWLabel(_x, _y, "T/V: ", map(engine_state.throttle_val, 0, 1023, 0, 500));
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

void setup_screen(){
  pinMode(BACKLIGHT_PIN, OUTPUT);
  analogWrite(BACKLIGHT_PIN, backlight_value);
  
  lcd.begin();
  lcd.setFont(u8g2_font_5x8_tf);

  lcd.setContrast(CONTRAST_SETTING);
  printSmiley(64, 32, 30);
  }