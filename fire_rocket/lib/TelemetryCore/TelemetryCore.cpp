#include "TelemetryCore.h"
#include <math.h>
#include <string.h>

namespace telemetry {
namespace {
int16_t i16(const uint8_t* p) {
    uint16_t u = uint16_t(p[0]) | (uint16_t(p[1]) << 8);
    return u < 0x8000 ? int16_t(u) : int16_t(int32_t(u) - 65536);
}
uint32_t u32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
int32_t i32(const uint8_t* p) {
    uint32_t u = u32(p);
    return u <= 0x7fffffff ? int32_t(u) : int32_t(int64_t(u) - 0x100000000LL);
}
const size_t widths[8] = {6, 6, 6, 6, 8, 2, 4, 4};
}

void Parser::reset() { *this = Parser(); }

void Parser::discard(size_t n) {
    memmove(buffer_, buffer_ + n, used_ - n);
    used_ -= n;
}

bool Parser::completeFrame(size_t at) const {
    if (used_ - at < 3 || buffer_[at] != 0xfa || buffer_[at + 1] != 0xfb) return false;
    const size_t length = buffer_[at + 2];
    const size_t total = length + 5;
    if (length < 2 || used_ - at < total) return false;
    if (buffer_[at + total - 2] != 0xfc || buffer_[at + total - 1] != 0xfd) return false;
    uint8_t sum = 0;
    for (size_t i = at + 3; i < at + total - 3; ++i) sum += buffer_[i];
    return sum == buffer_[at + total - 3];
}

bool Parser::feed(uint8_t byte, uint32_t now) {
    if (used_ == sizeof(buffer_)) { discard(1); ++counters_.framingErrors; }
    buffer_[used_++] = byte;
    bool updated = false;
    while (used_ >= 2) {
        if (buffer_[0] != 0xfa || buffer_[1] != 0xfb) { discard(1); continue; }
        if (used_ < 3) break;
        const size_t length = buffer_[2], total = length + 5;
        if (length < 2) { ++counters_.framingErrors; discard(1); continue; }
        if (used_ < total) {
            // A damaged length must not hold subsequent valid frames hostage.
            size_t next = 1;
            while (next + 7 <= used_ && !completeFrame(next)) ++next;
            if (next + 7 <= used_) { ++counters_.framingErrors; discard(next); continue; }
            break;
        }
        if (!completeFrame(0)) {
            if (buffer_[total - 2] != 0xfc || buffer_[total - 1] != 0xfd)
                ++counters_.framingErrors;
            else ++counters_.checksumErrors;
            discard(1);
            continue;
        }
        ++counters_.frames;
        updated = dispatch(buffer_[3], buffer_ + 4, length - 2, now) || updated;
        discard(total);
    }
    return updated;
}

bool Parser::dispatch(uint8_t cmd, const uint8_t* data, size_t n, uint32_t now) {
    if (cmd == 0x19) {
        if (n != 4 || data[1] < 1 || data[1] > 7 || data[2] < 1 || data[2] > 11 || data[3] > 1) {
            ++counters_.dataErrors; return false;
        }
        if (!config_.known || config_.fields != data[0] || config_.rate != data[1] ||
            config_.baud != data[2] || config_.reporting != data[3]) ++configRevision_;
        config_.known = true;
        config_.fields = data[0]; config_.rate = data[1];
        config_.baud = data[2]; config_.reporting = data[3];
        return false;
    }
    if (cmd != 0x00) return false; // ACKs never masquerade as samples.
    if (!config_.known) { ++counters_.missingConfig; return false; }
    if (n < 1 || (data[0] & 3) == 3) { ++counters_.dataErrors; return false; }
    Sample next;
    next.timeMs = now;
    next.type = data[0] & 3;
    next.magneticQuality = (data[0] >> 2) & 3;
    const uint8_t supported = next.type == 0 ? 0xff : (next.type == 1 ? 0x3f : 0x37);
    next.fields = config_.fields & supported;
    size_t expected = 1;
    for (unsigned i = 0; i < 8; ++i) if (next.fields & (1u << i)) expected += widths[i];
    if (n != expected) { ++counters_.dataErrors; return false; }
    const uint8_t* p = data + 1;
    auto vector = [&p](float* out, unsigned count, float scale) {
        for (unsigned j = 0; j < count; ++j, p += 2) out[j] = i16(p) * scale;
    };
    if (next.fields & Accel) vector(next.accel, 3, 0.00478515625f);
    if (next.fields & Gyro) vector(next.gyro, 3, 0.0625f);
    if (next.fields & Euler) vector(next.euler, 3, 0.0054931640625f);
    if (next.fields & Magnetic) vector(next.magnetic, 3, 0.006103515625f);
    if (next.fields & Quaternion) vector(next.quaternion, 4, 0.000030517578125f);
    if (next.fields & Temperature) { next.temperature = i16(p) * 0.01f; p += 2; }
    if (next.fields & Pressure) { next.pressurePa = u32(p) * 0.0002384185791f; p += 4; }
    if (next.fields & Altitude) next.altitudeM = i32(p) * 0.0010728836f;
    // Manufacturer ranges are validation limits, not control thresholds.
    if (((next.fields & Altitude) && (next.altitudeM < -500 || next.altitudeM > 9000)) ||
        ((next.fields & Pressure) && (next.pressurePa < 30000 || next.pressurePa > 120000)) ||
        ((next.fields & Temperature) && (next.temperature < -45 || next.temperature > 85))) {
        ++counters_.dataErrors; return false;
    }
    sample_ = next;
    ++counters_.samples;
    return true;
}

bool Settings::valid() const {
    return staleMs >= 100 && staleMs <= 30000 && recordSeconds >= 1 && recordSeconds <= 600 &&
        zeroWindowMs >= 500 && zeroWindowMs <= 10000 &&
        isfinite(zeroSpanM) && zeroSpanM > 0 && zeroSpanM <= 10;
}

void Monitor::reset() { *this = Monitor(); }
void Monitor::invalidate() { haveSample_ = zeroed_ = false; begin_ = count_ = 0; }

void Monitor::accept(const Sample& s, const Settings& cfg) {
    if (haveSample_ && (sample_.type != s.type || sample_.fields != s.fields)) invalidate();
    if (haveSample_ && uint32_t(s.timeMs - sample_.timeMs) > cfg.staleMs) count_ = begin_ = 0;
    sample_ = s; haveSample_ = true;
    if (!(s.fields & Altitude)) { count_ = begin_ = 0; return; }
    if (count_ == 256) { begin_ = (begin_ + 1) % 256; --count_; }
    heights_[(begin_ + count_) % 256] = {s.timeMs, s.altitudeM};
    ++count_;
    // Keep one boundary sample so that a complete window can be demonstrated.
    while (count_ > 2 && uint32_t(s.timeMs - heights_[(begin_ + 1) % 256].time) >= cfg.zeroWindowMs) {
        begin_ = (begin_ + 1) % 256; --count_;
    }
}

bool Monitor::fresh(uint32_t now, const Settings& cfg) const {
    return haveSample_ && uint32_t(now - sample_.timeMs) <= cfg.staleMs;
}

bool Monitor::canZero(uint32_t now, const Settings& cfg) const {
    if (!fresh(now, cfg) || !(sample_.fields & Altitude) || count_ < 2 ||
        uint32_t(sample_.timeMs - heights_[begin_].time) < cfg.zeroWindowMs) return false;
    float lo = heights_[begin_].value, hi = lo;
    for (size_t i = 1; i < count_; ++i) {
        float x = heights_[(begin_ + i) % 256].value;
        lo = fminf(lo, x); hi = fmaxf(hi, x);
    }
    return hi - lo <= cfg.zeroSpanM;
}

bool Monitor::zero(uint32_t now, const Settings& cfg) {
    if (!canZero(now, cfg)) return false;
    double sum = 0;
    for (size_t i = 0; i < count_; ++i) sum += heights_[(begin_ + i) % 256].value;
    baseline_ = float(sum / count_); zeroed_ = true;
    return true;
}

bool Monitor::relativeHeight(uint32_t now, const Settings& cfg, float& value) const {
    if (!zeroed_ || !fresh(now, cfg) || !(sample_.fields & Altitude)) return false;
    value = sample_.altitudeM - baseline_; return true;
}

bool Monitor::tilt(uint32_t now, const Settings& cfg, float& value) const {
    if (!fresh(now, cfg)) return false;
    float cosine;
    if (sample_.fields & Quaternion) {
        const float* q = sample_.quaternion;
        const float norm2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
        if (norm2 < 0.8f || norm2 > 1.2f) return false;
        cosine = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]) / norm2;
    } else if (sample_.fields & Euler) {
        constexpr float rad = 0.017453292519943f;
        cosine = cosf(sample_.euler[0] * rad) * cosf(sample_.euler[1] * rad);
    } else return false;
    value = acosf(fmaxf(-1, fminf(1, cosine))) * 57.295779513082f;
    return isfinite(value);
}

ButtonEvent Button::update(bool pressed, uint32_t now) {
    if (pressed != raw_) { raw_ = pressed; changed_ = now; }
    if (raw_ != stable_ && uint32_t(now - changed_) >= 40) {
        stable_ = raw_;
        if (stable_) { pressedAt_ = now; longSent_ = false; }
        else if (!longSent_) return ButtonEvent::Short;
    }
    if (stable_ && !longSent_ && uint32_t(now - pressedAt_) >= 2000) {
        longSent_ = true; return ButtonEvent::Long;
    }
    return ButtonEvent::None;
}
} // namespace telemetry
