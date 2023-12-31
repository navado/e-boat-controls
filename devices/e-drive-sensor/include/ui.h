#ifndef LCD_H
#define LCD_H


void draw_screen();
void printColumn1();
void printColumn2();
void printBigLabel();
void printSmileyFont(uint8_t index);
uint8_t printKWLabel(uint8_t x, uint8_t y, const char* key, uint32_t * value);
uint8_t printKWLabel(uint8_t x, uint8_t y, const char* key, char * value);
uint8_t printKWLabel(uint8_t x, uint8_t y, const char* key, String value);
uint8_t printKWLabel(uint8_t x, uint8_t y, const char* key, uint32_t value);
void printKWlabelKey(uint8_t x, uint8_t y, const char * key);
void setup_screen();
void display_serial_data();
#endif