#pragma once
#include <Arduino.h>
#include <FS.h>
#include <TelemetryCore.h>

struct LogRow {
    uint32_t timeMs;
    uint8_t fields, type, magneticQuality, validity;
    float accel[3], gyro[3], euler[3], magnetic[3], quaternion[4];
    float temperature, pressurePa, altitudeM, relativeM, tiltDeg, baselineM;
};
static_assert(sizeof(LogRow) == 96, "Log format v1 requires 96-byte rows");

class Recorder {
public:
    bool begin();
    bool format();
    bool start(uint32_t now, const telemetry::Settings& settings, float baseline, bool zeroed, bool replay);
    void append(const telemetry::Monitor& monitor, const telemetry::Settings& settings, uint32_t now);
    void event(uint32_t now, const char* code, uint32_t value);
    void tick(uint32_t now);
    void stop(const char* reason, uint32_t now);
    bool startExport(const char* name);
    void cancelExport();
    bool active() const { return active_; }
    bool exporting() const { return exportActive_; }
    bool mounted() const { return mounted_; }
    const char* status() const { return status_; }
    uint32_t rows() const { return rows_; }
private:
    File file_, events_, export_;
    bool mounted_ = false, active_ = false, exportCsv_ = false;
    bool exportActive_ = false, exportEnding_ = false;
    uint32_t started_ = 0, limitMs_ = 0, lastFlush_ = 0, rows_ = 0;
    const char* status_ = "NOT MOUNTED";
    LogRow buffer_[8];
    size_t count_ = 0;
    char tx_[640];
    size_t txSize_ = 0, txAt_ = 0;
    bool flush();
};
