#pragma once
const int U8G2_R0=0, U8X8_PIN_NONE=255;
static const unsigned char u8g2_font_5x7_tr[]={0};
class U8G2_SSD1306_72X40_ER_F_HW_I2C {
public:
    U8G2_SSD1306_72X40_ER_F_HW_I2C(int,int) {}
    void clearBuffer() {}
    void setFont(const unsigned char*) {}
    void drawStr(int,int,const char*) {}
    void sendBuffer() {}
    void setBusClock(unsigned) {}
    void begin() {}
    void setContrast(int) {}
};
