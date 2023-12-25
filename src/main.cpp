#include <Arduino.h>
#include <U8g2lib.h>
#include <TimeInterrupt.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


#define CS_PIN 8
#define RSE_PIN 7
#define RS_PIN 6
#define SCL_PIN 5
#define SDO_PIN 4
#define SDI_PIN 10
#define BACKLIGHT_PIN 9
#define LED_BRD_RD 2
#define LED_BRD_GR 3

#define CONTRAST_SETTING 0x3A

U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI lcd(U8G2_R0,
                                        /* clock=*/SCL_PIN,
                                        /* data=*/SDO_PIN,
                                        /* cs=*/CS_PIN,
                                        /* dc=*/RS_PIN,
                                        /* reset=*/RSE_PIN
                                        );

void setup() {
  Serial.begin(115200);
  // Prepare keepalive LED
  pinMode(LED_BRD_RD, OUTPUT);
  digitalWrite(LED_BRD_RD, HIGH);
  pinMode(LED_BRD_GR, OUTPUT);
  digitalWrite(LED_BRD_GR, LOW);
  
  // Prepare backlight
  analogWrite(BACKLIGHT_PIN, 30);
  
  lcd.begin();
  lcd.setFont(u8g2_font_5x8_tf);

  lcd.setContrast(CONTRAST_SETTING);
  Serial.println("LCD Init Done");
}

char buf[16]={0};

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

uint8_t frames = 0, fps = 0;
uint8_t current_pin = 0;
uint16_t cnt = 0;

void printColumn1(){
  uint8_t _x=2, _y = 8;
  _y = printKWLabel(_x, _y, "RPM: ", cnt * 60);
  _y = printKWLabel(_x, _y, "T: ", millis()/1000);
  _y = printKWLabel(_x, _y, "FPS: ", fps);
}

void printColumn2(){
  uint8_t _x=64, _y = 8;
  _y = printKWLabel(_x, _y, "CNT: ", cnt);
  // _y = printKWLabel(_x, _y, "F_M: ", FrequencyMeasured);
  // _y = printKWLabel(_x, _y, "D_M: ", DutycycleMeasured);
}

void printSmileyFont(uint8_t index){
  lcd.setFont(u8g2_font_emoticons21_tr);
  lcd.setCursor(2, 60);
  lcd.print(index);
}

void loop()
{
  digitalWrite(LED_BRD_GR, !digitalRead(LED_BRD_GR));
  digitalWrite(LED_BRD_RD, !digitalRead(LED_BRD_RD));

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