#include <Arduino.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TelemetryCore.h>
#include "Recorder.h"
#include <errno.h>
#include <cmath>
#include <stddef.h>

// Telemetry-only firmware: no arming, ignition, recovery or prediction outputs.
class AltitudeKalman {
public:
    void reset(float initZ = 0.0f) {
        z = initZ;
        vz = 0.0f;
        p00 = 1.0f; p01 = 0.0f;
        p10 = 0.0f; p11 = 1.0f;
        lastUpdateUs = 0;
        initialized = true;
    }

    void update(float z_baro, float az, float dt) {
        if (!initialized || dt <= 0.0f || dt > 0.2f) return;

        float z_pred  = z + vz * dt + 0.5f * az * dt * dt;
        float vz_pred = vz + az * dt;

        const float Q_accel = 1.0f;
        float q00 = 0.25f * dt * dt * dt * dt * Q_accel;
        float q01 = 0.5f * dt * dt * dt * Q_accel;
        float q11 = dt * dt * Q_accel;

        float p00_p = p00 + dt * (p01 + p10) + dt * dt * p11 + q00;
        float p01_p = p01 + dt * p11 + q01;
        float p10_p = p10 + dt * p11 + q01;
        float p11_p = p11 + q11;

        const float R_baro = 0.25f;
        float S = p00_p + R_baro;
        float K0 = p00_p / S;
        float K1 = p10_p / S;

        float y = z_baro - z_pred;
        z  = z_pred  + K0 * y;
        vz = vz_pred + K1 * y;

        p00 = (1.0f - K0) * p00_p;
        p01 = (1.0f - K0) * p01_p;
        p10 = p10_p - K1 * p00_p;
        p11 = p11_p - K1 * p01_p;
    }

    float altitude() const { return z; }
    float velocity() const { return vz; }
    bool  ready()    const { return initialized; }

private:
    float z = 0.0f;
    float vz = 0.0f;
    float p00 = 1.0f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
    uint32_t lastUpdateUs = 0;
    bool initialized = false;
};

namespace {
AltitudeKalman kf;
uint32_t lastKfUs = 0;
constexpr uint8_t buttonPin = 9;
constexpr uint8_t disabledOutputs[] = {0, 1, 7};
HardwareSerial sensor(1);
U8G2_SSD1306_72X40_ER_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
telemetry::Parser parser;
telemetry::Monitor monitor;
telemetry::Settings settings;
telemetry::Button button;
Recorder recorder;
Preferences preferences;
bool nvsReady = false, oledReady = false, replay = false, confirmation = false;
uint8_t page = 0;
uint32_t drawnAt = 0, queriedAt = 0, revision = 0, confirmedAt = 0;
const char* notice = "MONITOR ONLY";
uint32_t noticeAt = 0;
uint32_t lastDiagnosticAt = 0;
uint32_t diagnosticValues[8] = {};
bool diagnosticsInitialized = false;
uint32_t receivedBytes = 0, lastByteAt = 0, queryCount = 0;
uint8_t recentBytes[24] = {};
size_t recentCount = 0;
char input[600];
size_t inputSize = 0;
bool inputOverflow = false;

struct Persisted { uint32_t magic, version; telemetry::Settings settings; uint32_t crc; };
uint32_t checksum(const uint8_t* p, size_t n) {
    uint32_t x = 2166136261u;
    while (n--) x = (x ^ *p++) * 16777619u;
    return x;
}
void message(const char* text) { notice = text; noticeAt = millis(); }
void querySensor() {
    const uint8_t command[] = {0xfa,0xfb,3,0x19,1,0x1a,0xfc,0xfd};
    if (!replay) { sensor.write(command, sizeof(command)); ++queryCount; }
    queriedAt = millis();
}
void resetSource() {
    parser.reset(); monitor.reset(); revision = 0; confirmation = false;
    receivedBytes = lastByteAt = queryCount = 0; recentCount = 0;
    while (sensor.available()) sensor.read();
    querySensor();
}
void acceptByte(uint8_t byte, uint32_t now) {
    ++receivedBytes; lastByteAt = now;
    if (recentCount == sizeof(recentBytes)) {
        memmove(recentBytes, recentBytes + 1, sizeof(recentBytes)-1); --recentCount;
    }
    recentBytes[recentCount++] = byte;
    const bool updated = parser.feed(byte, now);
    if (parser.configRevision() != revision) {
        revision = parser.configRevision(); monitor.invalidate();
    }
    if (updated) {
        const auto& s = parser.sample();

        if (monitor.zeroed() && (s.fields & telemetry::Altitude)) {
            float rawRelHeight = s.altitudeM - monitor.baselineM();
            uint32_t currentUs = micros();
            if (!kf.ready()) {
                kf.reset(rawRelHeight);
                lastKfUs = currentUs;
            } else {
                float dt = (currentUs - lastKfUs) * 1e-6f;
                lastKfUs = currentUs;

                float q0 = s.quaternion[0];
                float q1 = s.quaternion[1];
                float q2 = s.quaternion[2];
                float q3 = s.quaternion[3];

                float ax = s.accel[0];
                float ay = s.accel[1];
                float az = s.accel[2];

                float a_nav_z = 2.0f * (q1 * q3 - q0 * q2) * ax +
                                2.0f * (q2 * q3 + q0 * q1) * ay +
                                (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az - 9.80665f;

                float accNorm = sqrtf(ax * ax + ay * ay + az * az);
                if (accNorm < 1.96f) {
                    a_nav_z = 0.0f;
                }

                kf.update(rawRelHeight, a_nav_z, dt);
                parser.mutableSample().altitudeM = kf.altitude() + monitor.baselineM();
            }
        } else {
            if (kf.ready()) kf.reset(0.0f);
        }

        monitor.accept(parser.sample(), settings);
        recorder.append(monitor, settings, now);
    }
}
bool zeroHeight() {
    if (recorder.active() || recorder.exporting()) return false;
    bool ok = monitor.zero(millis(), settings);
    if (ok) kf.reset(0.0f);
    return ok;
}
bool startRecording() {
    confirmation = false;
    if (!monitor.fresh(millis(), settings)) return false;
    bool ok = recorder.start(millis(), settings, monitor.baselineM(), monitor.zeroed(), replay);
    if (ok) diagnosticsInitialized = false;
    return ok;
}
void diagnostics(uint32_t now) {
    if (!recorder.active() || (diagnosticsInitialized && uint32_t(now-lastDiagnosticAt)<250)) return;
    lastDiagnosticAt = now;
    const auto& c = parser.counters();
    const uint32_t values[] = {
        uint32_t(monitor.fresh(now,settings)), c.checksumErrors, c.framingErrors, c.dataErrors,
        c.missingConfig, monitor.hasSample() ? uint32_t(monitor.sample().magneticQuality) : 255u,
        parser.config().fields, monitor.hasSample() ? uint32_t(monitor.sample().type) : 255u
    };
    const char* names[] = {"FRESH","BAD_CHECKSUM","BAD_FRAMING","BAD_DATA","NO_CONFIG","MAG_QUALITY","FIELD_MASK","SENSOR_TYPE"};
    for (unsigned i=0;i<8;++i) {
        if (!diagnosticsInitialized || values[i]!=diagnosticValues[i]) recorder.event(now,names[i],values[i]);
        diagnosticValues[i]=values[i];
    }
    diagnosticsInitialized = true;
}
void configuration() {
    Serial.printf("CFG stale_ms=%lu record_seconds=%lu zero_window_ms=%lu zero_span_m=%.3f\n",
        (unsigned long)settings.staleMs, (unsigned long)settings.recordSeconds,
        (unsigned long)settings.zeroWindowMs, settings.zeroSpanM);
}
const char* invalidReason(bool height, uint32_t now) {
    if (!parser.config().known) return receivedBytes ? "WAIT_CONFIG" : "NO_RX";
    if (!monitor.hasSample()) return "NO_SAMPLE";
    if (!monitor.fresh(now,settings)) return "STALE";
    const auto fields = monitor.sample().fields;
    if (height) {
        if (!(fields & telemetry::Altitude)) return "NO_ALTITUDE";
        if (!monitor.zeroed()) return "NOT_ZEROED";
    } else {
        if (!(fields & (telemetry::Quaternion|telemetry::Euler))) return "NO_ATTITUDE";
        float value;
        if (!monitor.tilt(now,settings,value)) return "BAD_ATTITUDE";
    }
    return "OK";
}
void rawValue(const char* name, float value, bool valid) {
    if (valid && std::isfinite(value)) Serial.printf(" %s=%.6f",name,double(value));
    else Serial.printf(" %s=NA",name);
}
void status() {
    const auto& c = parser.counters();
    const auto& cfg = parser.config();
    Serial.printf("STATUS mode=MONITOR_ONLY source=%s fresh=%d zeroed=%d recording=%d storage=%s\n",
        replay ? "REPLAY" : "LIVE", monitor.fresh(millis(), settings), monitor.zeroed(),
        recorder.active(), recorder.status());
    Serial.printf("SENSOR config=%d fields=%02X rate_code=%u reporting=%u samples=%lu checksum_errors=%lu framing_errors=%lu data_errors=%lu missing_config=%lu\n",
        cfg.known, cfg.fields, cfg.rate, cfg.reporting, (unsigned long)c.samples,
        (unsigned long)c.checksumErrors, (unsigned long)c.framingErrors,
        (unsigned long)c.dataErrors, (unsigned long)c.missingConfig);
    float h, tilt;
    if (monitor.relativeHeight(millis(), settings, h)) Serial.printf("HEIGHT %.3f m\n", h);
    else Serial.println("HEIGHT INVALID");
    if (monitor.tilt(millis(), settings, tilt)) Serial.printf("TILT %.3f deg\n", tilt);
    else Serial.println("TILT INVALID");
    if (kf.ready()) {
        Serial.printf("KF_HEIGHT=%.3f KF_VEL=%.3f\n", kf.altitude(), kf.velocity());
    }
    const uint32_t now = millis();
    Serial.printf("DIAG firmware=2 rx_bytes=%lu rx_age_ms=%ld queries=%lu height_reason=%s tilt_reason=%s zero_ready=%d\n",
        (unsigned long)receivedBytes, receivedBytes ? (long)uint32_t(now-lastByteAt) : -1L,
        (unsigned long)queryCount, invalidReason(true,now), invalidReason(false,now), monitor.canZero(now,settings));
    Serial.printf("RXHEX ");
    for (size_t i=0;i<recentCount;++i) Serial.printf("%02X",recentBytes[i]);
    Serial.println("");
    const auto& sample = monitor.sample();
    const bool fresh = monitor.fresh(now,settings);
    const uint8_t fields = fresh ? sample.fields : 0;
    Serial.printf("RAW type=%d fields=%02X mag_quality=%d age_ms=%ld",
        monitor.hasSample() ? int(sample.type) : -1,fields,
        monitor.hasSample() ? int(sample.magneticQuality) : -1,
        monitor.hasSample() ? (long)uint32_t(now-sample.timeMs) : -1L);
    rawValue("altitude_m",sample.altitudeM,fields&telemetry::Altitude);
    rawValue("pressure_pa",sample.pressurePa,fields&telemetry::Pressure);
    rawValue("temperature_c",sample.temperature,fields&telemetry::Temperature);
    const char* accNames[] = {"ax","ay","az"};
    const char* gyroNames[] = {"gx","gy","gz"};
    const char* angleNames[] = {"roll","pitch","yaw"};
    for (unsigned i=0;i<3;++i) {
        rawValue(accNames[i],sample.accel[i],fields&telemetry::Accel);
        rawValue(gyroNames[i],sample.gyro[i],fields&telemetry::Gyro);
        rawValue(angleNames[i],sample.euler[i],fields&telemetry::Euler);
    }
    Serial.println("");
}
bool setParameter(char* text) {
    char* separator = strchr(text, ' ');
    if (!separator) return false;
    *separator++ = 0;
    char* end = nullptr;
    errno = 0;
    const double v = strtod(separator, &end);
    if (errno || end == separator || *end || !std::isfinite(v)) return false;
    auto candidate = settings;
    if (strcmp(text, "zero_span_m") == 0) candidate.zeroSpanM = float(v);
    else {
        if (v < 0 || v > 30000 || floor(v) != v) return false;
        if (strcmp(text, "stale_ms") == 0) candidate.staleMs = uint32_t(v);
        else if (strcmp(text, "record_seconds") == 0) candidate.recordSeconds = uint32_t(v);
        else if (strcmp(text, "zero_window_ms") == 0) candidate.zeroWindowMs = uint32_t(v);
        else return false;
    }
    if (!candidate.valid()) return false;
    settings = candidate; monitor.invalidate(); confirmation = false; return true;
}
int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
void command(char* line) {
    if (recorder.exporting()) {
        if (strcmp(line,"EXPORT CANCEL") == 0) { recorder.cancelExport(); Serial.println("OK CANCELLED"); }
        return; // Do not splice status replies into an export stream.
    }
    if (!strcmp(line,"HELP")) {
        Serial.println("MONITOR ONLY; no actuator commands. STATUS | CFG GET | CFG SET key value | CFG SAVE | ZERO | SOURCE LIVE/REPLAY | RX hex | SENSOR QUERY | RECORD START/STOP | EXPORT CSV/META/END/EVENTS | FS FORMAT CONFIRM");
    } else if (!strcmp(line,"STATUS")) status();
    else if (!strcmp(line,"CFG GET")) configuration();
    else if (!strcmp(line,"SENSOR QUERY")) { querySensor(); Serial.println(replay ? "ERR REPLAY_MODE" : "OK QUERY_SENT"); }
    else if (!strcmp(line,"ZERO")) { const bool ok = zeroHeight(); message(ok ? "ZERO OK" : "ZERO REFUSED"); Serial.println(ok ? "OK ZERO" : "ERR ZERO_NEEDS_IDLE_STABLE_HEIGHT"); }
    else if (!strcmp(line,"RECORD START")) { bool ok = startRecording(); Serial.println(ok ? "OK RECORDING" : "ERR RECORD_START"); }
    else if (!strcmp(line,"RECORD STOP")) { recorder.stop("USER STOP", millis()); Serial.println("OK STOPPED"); }
    else if (!strncmp(line,"EXPORT ",7)) { if (!recorder.startExport(line+7)) Serial.println("ERR EXPORT"); }
    else if (!strncmp(line,"RX ",3)) {
        if (!replay) { Serial.println("ERR NOT_REPLAY"); return; }
        const size_t n = strlen(line+3);
        if (!n || n % 2 || n > 520) { Serial.println("ERR HEX_LENGTH"); return; }
        for (size_t i = 3; line[i]; ++i) if (hexDigit(line[i]) < 0) { Serial.println("ERR HEX"); return; }
        for (size_t i = 3; line[i]; i += 2) acceptByte(uint8_t(hexDigit(line[i])*16 + hexDigit(line[i+1])), millis());
        Serial.println("OK RX");
    } else if (recorder.active()) Serial.println("ERR CONFIG_LOCKED_WHILE_RECORDING");
    else if (!strcmp(line,"SOURCE LIVE") || !strcmp(line,"SOURCE REPLAY")) {
        replay = !strcmp(line,"SOURCE REPLAY"); resetSource(); Serial.println("OK SOURCE_RESET");
    } else if (!strncmp(line,"CFG SET ",8)) Serial.println(setParameter(line+8) ? "OK SET_REZERO_REQUIRED" : "ERR CONFIG");
    else if (!strcmp(line,"CFG SAVE")) {
        Persisted p = Persisted(); p.magic = 0x314e4f4d; p.version = 1; p.settings = settings;
        p.crc = checksum(reinterpret_cast<uint8_t*>(&p), offsetof(Persisted,crc));
        Serial.println(nvsReady && preferences.putBytes("settings", &p, sizeof(p)) == sizeof(p) ? "OK SAVED" : "ERR NVS");
    } else if (!strcmp(line,"FS FORMAT CONFIRM")) Serial.println(recorder.format() ? "OK FORMATTED" : "ERR FORMAT");
    else Serial.println("ERR UNKNOWN_COMMAND");
}
void usbInput() {
    unsigned budget = 768;
    while (budget-- && Serial.available()) {
        const char ch = char(Serial.read());
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (inputOverflow) { if (!recorder.exporting()) Serial.println("ERR LINE_TOO_LONG"); }
            else if (inputSize) { input[inputSize] = 0; command(input); }
            inputSize = 0; inputOverflow = false;
        } else if (inputSize < sizeof(input)-1 && !inputOverflow) input[inputSize++] = ch;
        else inputOverflow = true;
    }
}
void buttons(uint32_t now) {
    if (confirmation && uint32_t(now - confirmedAt) > 5000) confirmation = false;
    auto event = button.update(digitalRead(buttonPin) == LOW, now);
    if (recorder.exporting()) return;
    if (event == telemetry::ButtonEvent::Short) {
        if (confirmation) { confirmation = false; message(startRecording() ? "REC STARTED" : "REC REFUSED"); }
        else page = (page + 1) % 3;
    } else if (event == telemetry::ButtonEvent::Long) {
        if (recorder.active()) { recorder.stop("BUTTON STOP", now); message("REC STOPPED"); }
        else if (page == 2) { confirmation = true; confirmedAt = now; }
        else if (page == 0) message(zeroHeight() ? "ZERO OK" : "ZERO REFUSED");
    }
}
void draw(uint32_t now) {
    if (!oledReady || uint32_t(now - drawnAt) < 200) return;
    drawnAt = now;
    display.clearBuffer(); display.setFont(u8g2_font_5x7_tr);
    char line[32];
    if (page == 0) {
        if (kf.ready()) {
            snprintf(line, sizeof(line), "KF%6.1fm", kf.altitude());
            display.drawStr(0, 7, line);
            snprintf(line, sizeof(line), "Vz%6.1fm/s", kf.velocity());
            display.drawStr(0, 15, line);
        } else {
            display.drawStr(0, 7, "KF --");
            display.drawStr(0, 15, "Vz --");
        }
        const char* health = !parser.config().known ? (receivedBytes ? "WAIT CONFIG" : "NO UART RX") :
            !monitor.hasSample() ? "NO SAMPLE" : !monitor.fresh(now,settings) ? "STALE" :
            !monitor.zeroed() ? "NO ZERO" : "DATA OK";
        display.drawStr(0,23,health);
        display.drawStr(0,31,replay ? "REPLAY" : "LIVE");
    } else if (page == 1) {
        snprintf(line,sizeof(line),"RX %lu",(unsigned long)parser.counters().samples); display.drawStr(0,7,line);
        snprintf(line,sizeof(line),"ERR %lu",(unsigned long)(parser.counters().checksumErrors + parser.counters().dataErrors + parser.counters().framingErrors));
        display.drawStr(0,15,line);
        snprintf(line,sizeof(line),"MASK %02X",parser.config().fields); display.drawStr(0,23,line);
        if (!monitor.fresh(now,settings)) display.drawStr(0,31,"MAG --");
        else if (monitor.sample().type == 2) display.drawStr(0,31,"MAG N/A");
        else display.drawStr(0,31,monitor.sample().magneticQuality == 3 ? "MAG OK" : "MAG WARNING");
    } else {
        display.drawStr(0,7,"DATA RECORDER");
        display.drawStr(0,15,confirmation ? "SHORT:CONFIRM" : recorder.active() ? "LONG:STOP" : "LONG:NEW REC");
        snprintf(line,sizeof(line),"ROWS %lu",(unsigned long)recorder.rows()); display.drawStr(0,23,line);
        display.drawStr(0,31,confirmation ? "OVERWRITE OLD" : recorder.status());
    }
    display.drawStr(0,39,uint32_t(now - noticeAt) < 2000 ? notice : recorder.active() ? "RECORDING" : "MONITOR ONLY");
    display.sendBuffer();
}
}

void setup() {
    for (uint8_t pin : disabledOutputs) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
    pinMode(buttonPin,INPUT_PULLUP);
    Serial.begin(115200); // Never wait for a USB host.
    nvsReady = preferences.begin("fr-monitor",false);
    Persisted p = Persisted();
    if (nvsReady && preferences.getBytesLength("settings") == sizeof(p) &&
        preferences.getBytes("settings",&p,sizeof(p)) == sizeof(p) &&
        p.magic == 0x314e4f4d && p.version == 1 && p.settings.valid() &&
        p.crc == checksum(reinterpret_cast<uint8_t*>(&p),offsetof(Persisted,crc))) settings = p.settings;
    recorder.begin();
    sensor.setRxBufferSize(2048);
    sensor.begin(115200,SERIAL_8N1,3,4);
    Wire.begin(5,6); Wire.setTimeOut(20);
    Wire.beginTransmission(0x3c);
    oledReady = Wire.endTransmission() == 0;
    if (oledReady) { display.setBusClock(400000); display.begin(); display.setContrast(160); }
    querySensor();
}

void loop() {
    const uint32_t now = millis();
    unsigned budget = 512;
    while (budget-- && sensor.available()) {
        uint8_t b = uint8_t(sensor.read());
        if (!replay) acceptByte(b,now);
    }
    if (!replay && uint32_t(now - queriedAt) >= 2000) querySensor();
    usbInput(); buttons(millis()); diagnostics(millis()); recorder.tick(millis()); draw(millis());
    delay(1);
}
