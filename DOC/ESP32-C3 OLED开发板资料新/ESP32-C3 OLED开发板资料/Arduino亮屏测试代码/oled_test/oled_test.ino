#include <U8g2lib.h>
#include <Wire.h>

#define led 8

//# IIC version
U8G2_SSD1306_72X40_ER_1_SW_I2C u8g2(U8G2_R0,6,5,U8X8_PIN_NONE); //ESP32C3 OLED开发板的屏幕接线：scl-->gpio6  sda-->gpio5 如果是Arduino则改为scl-->A5  sda-->A4


void setup(void) {
  pinMode(led, OUTPUT);
 u8g2.setContrast(250);
 u8g2.begin();
}
void loop(void) {
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
 u8g2.firstPage();
 do {
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(2,15,"ESP32-C3");
 } while ( u8g2.nextPage() );
}
