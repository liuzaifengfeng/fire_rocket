#include "Arduino.h"
#include "LittleFS.h"
#include "Wire.h"
#include <cmath>
#include <vector>
#include <string>
#include <limits>

uint32_t fakeNow=0;
int mockPins[32]={};
size_t mockWriteLimit=std::numeric_limits<size_t>::max();
HardwareSerial Serial;
MockLittleFS LittleFS;
MockWire Wire;

// Include the real application so command and button paths are tested unchanged.
#include "../../src/main.cpp"

static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) { std::fprintf(stderr,"FAIL integration line %d: %s\n",__LINE__,#x); std::exit(1); } } while(0)
static std::string call(const std::string& text) {
    Serial.tx.clear(); std::vector<char> line(text.begin(),text.end()); line.push_back(0);
    command(line.data()); return Serial.tx;
}
static void sensorSample(uint32_t t) {
    telemetry::Sample s;
    s.timeMs=t; s.fields=telemetry::Altitude|telemetry::Quaternion;
    s.altitudeM=100; s.quaternion[0]=1;
    monitor.accept(s,settings); fakeNow=t;
}
static void drain() {
    for(unsigned n=0;recorder.exporting()&&n<20000;++n) recorder.tick(fakeNow++);
    CHECK(!recorder.exporting());
}
int main() {
    mockPins[9]=HIGH;
    setup();
    CHECK(mockPins[0]==LOW&&mockPins[1]==LOW&&mockPins[7]==LOW);
    CHECK(!recorder.mounted()); CHECK(!recorder.active());
    std::string initialStatus=call("STATUS");
    CHECK(initialStatus.find("firmware=2 rx_bytes=0")!=std::string::npos);
    CHECK(initialStatus.find("height_reason=NO_RX tilt_reason=NO_RX")!=std::string::npos);
    CHECK(initialStatus.find("altitude_m=NA")!=std::string::npos);
    CHECK(call("ZERO").find("ERR")!=std::string::npos);
    CHECK(call("FS FORMAT CONFIRM").find("OK")!=std::string::npos);
    CHECK(recorder.mounted());
    CHECK(call("CFG SET stale_ms nan").find("ERR")!=std::string::npos);
    CHECK(call("CFG SET stale_ms 500x").find("ERR")!=std::string::npos);
    CHECK(call("CFG SET stale_ms -1").find("ERR")!=std::string::npos);
    CHECK(call("CFG SET stale_ms 500.5").find("ERR")!=std::string::npos);
    CHECK(call("CFG SET zero_span_m 0.25").find("OK")!=std::string::npos);
    CHECK(call("CFG SAVE").find("OK")!=std::string::npos);
    CHECK(preferences.getBytesLength("settings")==sizeof(Persisted));
    for(uint32_t t=0;t<=2000;t+=50) sensorSample(t);
    CHECK(call("ZERO").find("OK ZERO")!=std::string::npos);
    CHECK(monitor.zeroed());
    CHECK(call("RECORD START").find("OK")!=std::string::npos);
    CHECK(recorder.active());
    diagnostics(fakeNow); diagnostics(fakeNow+600);
    CHECK(call("CFG SET stale_ms 1000").find("LOCKED")!=std::string::npos);
    CHECK(settings.staleMs==500);
    CHECK(call("SOURCE REPLAY").find("LOCKED")!=std::string::npos);
    CHECK(call("ZERO").find("ERR")!=std::string::npos);
    CHECK(call("EXPORT CSV").find("ERR")!=std::string::npos);
    for(unsigned i=1;i<=12;++i) { sensorSample(2600+i*50); recorder.append(monitor,settings,fakeNow); }
    CHECK(call("RECORD STOP").find("OK")!=std::string::npos);
    CHECK(LittleFS.files["/samples.bin"]->size()==8+12*sizeof(LogRow));
    LogRow record;
    memcpy(&record,LittleFS.files["/samples.bin"]->data()+8,sizeof(record));
    CHECK(std::isnan(record.accel[0])); CHECK(record.relativeM==0); CHECK(record.validity==7);
    CHECK(call("EXPORT CSV").empty());
    Serial.capacity=0; recorder.tick(fakeNow); CHECK(Serial.tx.empty()); CHECK(recorder.exporting());
    Serial.capacity=5; drain();
    CHECK(Serial.tx.find("BEGIN CSV\n") == 0);
    CHECK(Serial.tx.find("nan")!=std::string::npos);
    CHECK(Serial.tx.substr(Serial.tx.size()-4)=="END\n");
    Serial.capacity=65536;
    CHECK(call("EXPORT META").empty()); drain(); CHECK(Serial.tx.find("monitor_only")!=std::string::npos);
    CHECK(call("EXPORT EVENTS").empty()); drain();
    CHECK(Serial.tx.find("2000,FRESH,1")!=std::string::npos);
    CHECK(Serial.tx.find("2600,FRESH,0")!=std::string::npos);
    CHECK(call("EXPORT CSV").empty()); Serial.connected=false; recorder.tick(fakeNow);
    CHECK(!recorder.exporting()); Serial.connected=true;
    // A damaged trailing row is reported rather than converted to a bogus row.
    LittleFS.files["/samples.bin"]->push_back(0xaa);
    call("EXPORT CSV"); drain(); CHECK(Serial.tx.find("ERR TRUNCATED_LOG")!=std::string::npos);
    CHECK(call("SOURCE REPLAY").find("OK")!=std::string::npos);
    CHECK(!monitor.zeroed()); CHECK(!monitor.hasSample());
    CHECK(call("RX zz").find("ERR HEX")!=std::string::npos);
    CHECK(call("RX fafb061980070601a7fcfd").find("OK RX")!=std::string::npos);
    // Valid configuration and altitude-only sample generated explicitly.
    CHECK(parser.config().known); CHECK(parser.config().fields==128);
    CHECK(call("RX fafb07000c176c010090fcfd").find("OK RX")!=std::string::npos);
    CHECK(monitor.hasSample());
    std::string altitudeOnly=call("STATUS");
    CHECK(altitudeOnly.find("height_reason=NOT_ZEROED tilt_reason=NO_ATTITUDE")!=std::string::npos);
    CHECK(altitudeOnly.find("altitude_m=100.")!=std::string::npos);
    CHECK(call("RECORD START").find("OK")!=std::string::npos);
    // Replay injection remains available while recording, unlike configuration writes.
    CHECK(call("RX fafb07000c176c010090fcfd").find("OK RX")!=std::string::npos);
    CHECK(recorder.rows()==1);
    CHECK(call("RECORD STOP").find("OK")!=std::string::npos);
    CHECK(LittleFS.files["/samples.bin"]->size()==8+sizeof(LogRow)); // replaces previous session
    CHECK(call("CFG SET record_seconds 1").find("OK")!=std::string::npos);
    sensorSample(fakeNow); CHECK(startRecording());
    recorder.tick(fakeNow+1000); CHECK(!recorder.active());
    CHECK(std::string(recorder.status())=="TIME LIMIT");
    LittleFS.capacity=1;
    CHECK(!startRecording()); CHECK(std::string(recorder.status())=="NO SPACE");
    LittleFS.capacity=0x270000;
    sensorSample(fakeNow); CHECK(startRecording());
    recorder.append(monitor,settings,fakeNow); mockWriteLimit=1;
    recorder.stop("USER STOP",fakeNow); CHECK(std::string(recorder.status())=="WRITE ERROR");
    mockWriteLimit=std::numeric_limits<size_t>::max();
    // Overlong USB lines are discarded all the way through their newline.
    std::string overlong(650,'x'); overlong+="STATUS\nSTATUS\n";
    for(char ch:overlong) Serial.rx.push_back(uint8_t(ch));
    Serial.tx.clear(); usbInput();
    CHECK(Serial.tx.find("ERR LINE_TOO_LONG")!=std::string::npos);
    CHECK(Serial.tx.find("STATUS mode=MONITOR_ONLY")!=std::string::npos);
    CHECK(call("ARM").find("ERR UNKNOWN_COMMAND")!=std::string::npos);
    // Real application button path: hold, release, short-confirm, then hold-stop.
    CHECK(call("CFG SET record_seconds 600").find("OK")!=std::string::npos);
    page=2;
    auto press = [](uint32_t t, bool down) { fakeNow=t; mockPins[9]=down?LOW:HIGH; buttons(t); };
    press(20000,true); press(20040,true); press(22040,true);
    CHECK(confirmation); CHECK(!recorder.active());
    press(22100,false); press(22140,false);
    CHECK(confirmation); CHECK(!recorder.active());
    press(22200,true); press(22240,true); press(22300,false);
    sensorSample(22340); press(22340,false);
    CHECK(recorder.active()); CHECK(!confirmation);
    press(22400,true); press(22440,true); press(24440,true);
    CHECK(!recorder.active()); CHECK(std::string(recorder.status())=="BUTTON STOP");
    press(24500,false); press(24540,false); CHECK(!confirmation);
    CHECK(mockPins[0]==LOW&&mockPins[1]==LOW&&mockPins[7]==LOW);
    std::printf("PASS application and recorder: %u assertions (mock hardware/filesystem, no device test)\n",checks);
}
