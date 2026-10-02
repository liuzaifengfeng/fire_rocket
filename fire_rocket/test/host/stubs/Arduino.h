#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <deque>
#include <string>
#include <algorithm>
const int LOW=0, HIGH=1, OUTPUT=1, INPUT_PULLUP=2, SERIAL_8N1=0;
extern uint32_t fakeNow;
extern int mockPins[32];
inline uint32_t millis() { return fakeNow; }
inline uint32_t micros() { return fakeNow * 1000; }
inline void delay(unsigned ms) { fakeNow+=ms; }
inline int digitalRead(int pin) { return mockPins[pin]; }
inline void digitalWrite(int pin,int v) { mockPins[pin]=v; }
inline void pinMode(int,int) {}
class HardwareSerial {
public:
    explicit HardwareSerial(int=0) {}
    bool connected=true;
    size_t capacity=65536;
    std::string tx;
    std::deque<uint8_t> rx;
    void begin(unsigned,int=0,int=0,int=0) {}
    void setRxBufferSize(size_t) {}
    explicit operator bool() const { return connected; }
    int available() const { return int(rx.size()); }
    int read() { if(rx.empty()) return -1; int v=rx.front(); rx.pop_front(); return v; }
    size_t availableForWrite() const { return capacity; }
    size_t write(const uint8_t* b,size_t n) { n=std::min(n,capacity); tx.append(reinterpret_cast<const char*>(b),n); return n; }
    size_t println(const char* s) { std::string v=std::string(s)+"\n"; return write(reinterpret_cast<const uint8_t*>(v.data()),v.size()); }
    size_t printf(const char* fmt,...) {
        char b[2048]; va_list args; va_start(args,fmt); int n=vsnprintf(b,sizeof(b),fmt,args); va_end(args);
        return write(reinterpret_cast<uint8_t*>(b),size_t(std::max(0,n)));
    }
};
extern HardwareSerial Serial;
