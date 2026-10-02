#include "TelemetryCore.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <limits>

using namespace telemetry;
typedef std::vector<uint8_t> Bytes;
static unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); std::exit(1); } } while (0)
static void close(float a, float b, float eps = 0.001f) { CHECK(std::fabs(a-b) < eps); }
static Bytes frame(uint8_t cmd, Bytes data) {
    Bytes out = {0xfa,0xfb,uint8_t(data.size()+2),cmd};
    uint8_t sum = cmd;
    for (uint8_t v : data) { out.push_back(v); sum += v; }
    out.push_back(sum); out.push_back(0xfc); out.push_back(0xfd);
    return out;
}
static bool feed(Parser& p, const Bytes& bytes, uint32_t now=10) {
    bool result=false;
    for (uint8_t b : bytes) result=p.feed(b,now)||result;
    return result;
}
static void append16(Bytes& b,int16_t v) { b.push_back(uint16_t(v)&255); b.push_back(uint16_t(v)>>8); }
static void append32(Bytes& b,int32_t v) { for (unsigned s=0;s<32;s+=8) b.push_back(uint32_t(v)>>s); }
static Bytes report(uint8_t mask=255,uint8_t type=0,int32_t altitude=93207) {
    Bytes b = {uint8_t(type|12)};
    const uint8_t supported=type==0?255:(type==1?63:55);
    mask &= supported;
    if (mask&Accel) { append16(b,2048); append16(b,-2048); append16(b,0); }
    if (mask&Gyro) { append16(b,-16); append16(b,32); append16(b,0); }
    if (mask&Euler) { append16(b,0); append16(b,16384); append16(b,-16384); }
    if (mask&Magnetic) { append16(b,16384); append16(b,-16384); append16(b,0); }
    if (mask&Quaternion) { append16(b,32767); append16(b,0); append16(b,0); append16(b,0); }
    if (mask&Temperature) append16(b,-1200);
    if (mask&Pressure) append32(b,419430400);
    if (mask&Altitude) append32(b,altitude);
    return frame(0,b);
}
static void config(Parser& p,uint8_t fields=255) { feed(p,frame(0x19,{fields,7,6,1})); }

static void protocol() {
    Parser p;
    CHECK(!feed(p,report())); CHECK(p.counters().missingConfig==1);
    config(p); CHECK(p.config().known); CHECK(p.configRevision()==1);
    config(p); CHECK(p.configRevision()==1);
    CHECK(feed(p,report(),1234)); CHECK(p.sample().timeMs==1234);
    close(p.sample().accel[0],9.8f); close(p.sample().accel[1],-9.8f);
    close(p.sample().gyro[0],-1); close(p.sample().euler[1],90);
    close(p.sample().magnetic[0],100); close(p.sample().temperature,-12);
    close(p.sample().pressurePa,100000,0.02f); close(p.sample().altitudeM,100,0.002f);
    CHECK(p.sample().magneticQuality==3); CHECK(p.sample().fields==255);
    CHECK(feed(p,report(255,0,-93207))); close(p.sample().altitudeM,-100,0.002f);
    CHECK(!feed(p,report(255,0,20000000))); CHECK(p.counters().dataErrors==1);
    CHECK(!feed(p,frame(0x16,{1}))); // A successful sensor ACK is not telemetry.
    for (unsigned type=0;type<3;++type) for (unsigned mask=0;mask<256;++mask) {
        p.reset(); config(p,uint8_t(mask));
        CHECK(feed(p,report(uint8_t(mask),uint8_t(type))));
        CHECK(p.sample().fields==(mask&(type==0?255:type==1?63:55)));
        CHECK(p.sample().type==type);
    }
    p.reset(); config(p); CHECK(!feed(p,frame(0,{3})));
    p.reset(); CHECK(!feed(p,frame(0x19,{255,8,6,1}))); CHECK(!p.config().known);
    config(p,Altitude); CHECK(!feed(p,report())); CHECK(p.counters().samples==0);
    CHECK(feed(p,report(Altitude))); CHECK(p.sample().fields==Altitude);
    std::puts("PASS protocol: units, signed values, 768 type/subscription combinations, ACK isolation");
}

static void corruption() {
    Parser p; config(p);
    Bytes bad=report(); bad[bad.size()-3]^=1;
    CHECK(!feed(p,bad)); CHECK(p.counters().checksumErrors==1);
    CHECK(feed(p,report()));
    bad=report(); bad.back()=0; CHECK(!feed(p,bad)); CHECK(feed(p,report()));
    feed(p,{0x33,0xfa,0xfa,0xfb,0xff,0x44}); // Broken length plus embedded valid frame.
    CHECK(feed(p,report()));
    feed(p,{0xfa,0xfb,0,0xfc,0xfd}); CHECK(feed(p,report()));
    bad=report(); for (size_t i=0;i<bad.size()-1;++i) CHECK(!p.feed(bad[i],20));
    CHECK(p.feed(bad.back(),20));
    uint32_t state=1234;
    for (unsigned i=0;i<100000;++i) { state=state*1664525u+1013904223u; p.feed(uint8_t(state>>24),30); }
    CHECK(feed(p,report()));
    std::puts("PASS framing: split packets, corrupt checksum/tail/length, resync, 100000 noise bytes");
}

static Sample s(uint32_t t,float altitude=100) {
    Sample x; x.timeMs=t; x.fields=Altitude|Quaternion|Accel;
    x.altitudeM=altitude; x.quaternion[0]=1; x.accel[2]=9.8f; return x;
}
static void monitoring() {
    Monitor m; Settings cfg; float value=0;
    CHECK(!m.fresh(0,cfg)); CHECK(!m.zero(0,cfg)); CHECK(!m.relativeHeight(0,cfg,value));
    for (unsigned t=0;t<=2000;t+=50) m.accept(s(t,100+0.1f*float((t/50)%2)),cfg);
    CHECK(m.canZero(2000,cfg)); CHECK(m.zero(2000,cfg));
    CHECK(m.relativeHeight(2000,cfg,value)); CHECK(std::fabs(value)<0.1f);
    CHECK(m.tilt(2000,cfg,value)); close(value,0);
    CHECK(m.fresh(2500,cfg)); CHECK(!m.fresh(2501,cfg));
    CHECK(!m.relativeHeight(2501,cfg,value)); CHECK(!m.tilt(2501,cfg,value));
    m.accept(s(4000),cfg); CHECK(!m.canZero(4000,cfg));
    m.reset();
    for (unsigned t=0;t<=2000;t+=50) m.accept(s(t,float(t)),cfg);
    CHECK(!m.canZero(2000,cfg));
    auto x=s(2100); x.quaternion[0]=0; x.quaternion[1]=1;
    m.accept(x,cfg); CHECK(m.tilt(2100,cfg,value)); close(value,180);
    x.quaternion[1]=0; m.accept(x,cfg); CHECK(!m.tilt(2100,cfg,value));
    x.fields=Euler; x.euler[0]=90; m.accept(x,cfg);
    CHECK(m.tilt(2100,cfg,value)); close(value,90); CHECK(!m.zeroed());
    CHECK(!m.relativeHeight(2100,cfg,value));
    m.reset(); m.accept(s(0xfffffff0u),cfg);
    CHECK(m.fresh(0x20,cfg)); CHECK(!m.fresh(0x300,cfg));
    cfg.zeroWindowMs=10000; m.reset();
    for (unsigned t=0;t<=20000;t+=50) m.accept(s(t),cfg);
    CHECK(m.canZero(20000,cfg)); // ring buffer holds a complete 10s window at 20Hz.
    CHECK(cfg.valid()); cfg.recordSeconds=0; CHECK(!cfg.valid());
    cfg=Settings(); cfg.zeroSpanM=std::numeric_limits<float>::quiet_NaN(); CHECK(!cfg.valid());
    std::puts("PASS monitoring: stable zero, stale fields, tilt, type changes, millis rollover, ring window");
}

static void buttons() {
    Button b;
    CHECK(b.update(true,0)==ButtonEvent::None);
    CHECK(b.update(false,10)==ButtonEvent::None);
    CHECK(b.update(false,100)==ButtonEvent::None);
    CHECK(b.update(true,200)==ButtonEvent::None);
    CHECK(b.update(true,240)==ButtonEvent::None);
    CHECK(b.update(false,300)==ButtonEvent::None);
    CHECK(b.update(false,340)==ButtonEvent::Short);
    CHECK(b.update(true,500)==ButtonEvent::None);
    CHECK(b.update(true,540)==ButtonEvent::None);
    CHECK(b.update(true,2540)==ButtonEvent::Long);
    CHECK(b.update(true,2600)==ButtonEvent::None);
    CHECK(b.update(false,2700)==ButtonEvent::None);
    CHECK(b.update(false,2740)==ButtonEvent::None);
    std::puts("PASS buttons: debounce, short/long exclusivity, no repeat");
}
int main() { protocol(); corruption(); monitoring(); buttons(); std::printf("PASS %u assertions\n",checks); }
