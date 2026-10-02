#pragma once

#include <stddef.h>
#include <stdint.h>

namespace telemetry {

enum Field : uint8_t {
    Accel = 1, Gyro = 2, Euler = 4, Magnetic = 8,
    Quaternion = 16, Temperature = 32, Pressure = 64, Altitude = 128
};

struct Sample {
    uint32_t timeMs = 0;
    uint8_t type = 0, magneticQuality = 0, fields = 0;
    float accel[3] = {}, gyro[3] = {}, euler[3] = {}, magnetic[3] = {};
    float quaternion[4] = {}, temperature = 0, pressurePa = 0, altitudeM = 0;
};

struct SensorConfig {
    bool known = false;
    uint8_t fields = 0, rate = 0, baud = 0, reporting = 0;
};

struct Counters {
    uint32_t frames = 0, samples = 0, checksumErrors = 0;
    uint32_t framingErrors = 0, dataErrors = 0, missingConfig = 0;
};

class Parser {
public:
    // True only when this byte completes a fully validated data sample.
    bool feed(uint8_t byte, uint32_t now);
    void reset();
    const Sample& sample() const { return sample_; }
    Sample& mutableSample() { return sample_; }
    const SensorConfig& config() const { return config_; }
    const Counters& counters() const { return counters_; }
    uint32_t configRevision() const { return configRevision_; }
private:
    uint8_t buffer_[260] = {};
    size_t used_ = 0;
    Sample sample_;
    SensorConfig config_;
    Counters counters_;
    uint32_t configRevision_ = 0;
    void discard(size_t n);
    bool completeFrame(size_t at) const;
    bool dispatch(uint8_t command, const uint8_t* data, size_t n, uint32_t now);
};

struct Settings {
    uint32_t staleMs = 500;       // Monitor indication only, not flight control.
    uint32_t recordSeconds = 600;
    uint32_t zeroWindowMs = 2000;
    float zeroSpanM = 0.5f;
    bool valid() const;
};

class Monitor {
public:
    void accept(const Sample& sample, const Settings& cfg);
    void reset();
    void invalidate();
    bool fresh(uint32_t now, const Settings& cfg) const;
    bool canZero(uint32_t now, const Settings& cfg) const;
    bool zero(uint32_t now, const Settings& cfg);
    bool relativeHeight(uint32_t now, const Settings& cfg, float& value) const;
    bool tilt(uint32_t now, const Settings& cfg, float& value) const;
    bool zeroed() const { return zeroed_; }
    float baselineM() const { return baseline_; }
    bool hasSample() const { return haveSample_; }
    const Sample& sample() const { return sample_; }
private:
    struct Height { uint32_t time; float value; };
    Height heights_[256] = {};
    size_t begin_ = 0, count_ = 0;
    Sample sample_;
    bool haveSample_ = false, zeroed_ = false;
    float baseline_ = 0;
};

enum class ButtonEvent { None, Short, Long };
class Button {
public:
    // Active-low input is converted to pressed=true by the hardware adapter.
    ButtonEvent update(bool pressed, uint32_t now);
private:
    bool raw_ = false, stable_ = false, longSent_ = false;
    uint32_t changed_ = 0, pressedAt_ = 0;
};

} // namespace telemetry
