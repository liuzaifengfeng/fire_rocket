#pragma once
class MockWire {
public:
    void begin(int,int) {}
    void setTimeOut(int) {}
    void beginTransmission(int) {}
    int endTransmission() { return 0; }
};
extern MockWire Wire;
