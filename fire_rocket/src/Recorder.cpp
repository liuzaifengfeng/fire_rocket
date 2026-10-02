#include "Recorder.h"
#include <LittleFS.h>
#include <math.h>
#include <string.h>

namespace {
const char header[] = "FRMON01\n";
const char csvHeader[] = "t_ms,fields,type,mag_quality,validity,ax,ay,az,gx,gy,gz,roll,pitch,yaw,mx,my,mz,qw,qx,qy,qz,temp_c,pressure_pa,altitude_m,relative_m,tilt_deg,baseline_m\n";
}

bool Recorder::begin() {
    mounted_ = LittleFS.begin(false); // Never silently erase previous records.
    status_ = mounted_ ? "IDLE" : "NEED FORMAT";
    return mounted_;
}

bool Recorder::format() {
    if (active_ || exportActive_) return false;
    LittleFS.end();
    mounted_ = false;
    if (!LittleFS.format()) { status_ = "FORMAT ERROR"; return false; }
    return begin();
}

bool Recorder::start(uint32_t now, const telemetry::Settings& cfg, float baseline, bool zeroed, bool replay) {
    if (!mounted_ || active_ || exportActive_ || !cfg.valid()) return false;
    // Space is checked before replacing the previous session.
    size_t reclaimed = 0;
    const char* oldPaths[] = {"/samples.bin", "/session.json", "/end.json", "/events.csv"};
    for (const char* path : oldPaths) {
        File old = LittleFS.open(path, "r");
        if (old) reclaimed += old.size();
        old.close();
    }
    // Reserve space for sample records and rate-limited diagnostic events.
    const size_t required = size_t(cfg.recordSeconds) * (20 * sizeof(LogRow) + 2000) + 65536;
    const size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
    const size_t free = total > used ? total - used : 0;
    if (free + reclaimed < required) {
        status_ = "NO SPACE"; return false;
    }
    file_ = LittleFS.open("/samples.bin", "w");
    if (!file_ || file_.write(reinterpret_cast<const uint8_t*>(header), sizeof(header)-1) != sizeof(header)-1) {
        file_.close(); status_ = "WRITE ERROR"; return false;
    }
    File meta = LittleFS.open("/session.json", "w");
    if (!meta) { file_.close(); status_ = "META ERROR"; return false; }
    const size_t wrote = meta.printf(
        "{\"format\":1,\"row_bytes\":96,\"source\":\"%s\",\"monitor_only\":true,"
        "\"start_ms\":%lu,\"stale_ms\":%lu,\"record_seconds\":%lu,\"zero_window_ms\":%lu,"
        "\"zero_span_m\":%.6f,\"zeroed\":%s,\"baseline_m\":%.6f}\n",
        replay ? "REPLAY" : "LIVE", (unsigned long)now, (unsigned long)cfg.staleMs,
        (unsigned long)cfg.recordSeconds, (unsigned long)cfg.zeroWindowMs,
        cfg.zeroSpanM, zeroed ? "true" : "false", baseline);
    meta.close();
    if (!wrote) { file_.close(); status_ = "META ERROR"; return false; }
    events_ = LittleFS.open("/events.csv", "w");
    const char eventHeader[] = "t_ms,event,value\n";
    if (!events_ || events_.write(reinterpret_cast<const uint8_t*>(eventHeader), sizeof(eventHeader)-1) != sizeof(eventHeader)-1) {
        events_.close(); file_.close(); status_ = "EVENTS ERROR"; return false;
    }
    LittleFS.remove("/end.json");
    active_ = true; started_ = lastFlush_ = now;
    limitMs_ = cfg.recordSeconds * 1000; rows_ = 0; count_ = 0; status_ = "RECORDING";
    return true;
}

void Recorder::event(uint32_t now, const char* code, uint32_t value) {
    if (!active_) return;
    char line[80];
    const int n = snprintf(line, sizeof(line), "%lu,%s,%lu\n", (unsigned long)now, code, (unsigned long)value);
    if (n <= 0 || size_t(n) >= sizeof(line) ||
        events_.write(reinterpret_cast<const uint8_t*>(line), size_t(n)) != size_t(n)) stop("EVENTS ERROR", now);
}

bool Recorder::flush() {
    if (!count_) return true;
    const size_t bytes = count_ * sizeof(LogRow);
    const bool ok = file_.write(reinterpret_cast<const uint8_t*>(buffer_), bytes) == bytes;
    // A partial write is terminal; never append the same block again.
    count_ = 0;
    return ok;
}

void Recorder::append(const telemetry::Monitor& monitor, const telemetry::Settings& cfg, uint32_t now) {
    if (!active_) return;
    if (uint32_t(now - started_) >= limitMs_) { stop("TIME LIMIT", now); return; }
    const auto& s = monitor.sample();
    LogRow row = LogRow();
    row.timeMs = s.timeMs; row.fields = s.fields; row.type = s.type;
    row.magneticQuality = s.magneticQuality;
    row.validity = monitor.fresh(now, cfg) ? 1 : 0;
    memcpy(row.accel, s.accel, sizeof(row.accel));
    memcpy(row.gyro, s.gyro, sizeof(row.gyro));
    memcpy(row.euler, s.euler, sizeof(row.euler));
    memcpy(row.magnetic, s.magnetic, sizeof(row.magnetic));
    memcpy(row.quaternion, s.quaternion, sizeof(row.quaternion));
    // Absent raw fields remain NaN in both binary and CSV records.
    if (!(s.fields & telemetry::Accel)) for (float& v : row.accel) v = NAN;
    if (!(s.fields & telemetry::Gyro)) for (float& v : row.gyro) v = NAN;
    if (!(s.fields & telemetry::Euler)) for (float& v : row.euler) v = NAN;
    if (!(s.fields & telemetry::Magnetic)) for (float& v : row.magnetic) v = NAN;
    if (!(s.fields & telemetry::Quaternion)) for (float& v : row.quaternion) v = NAN;
    row.temperature = s.fields & telemetry::Temperature ? s.temperature : NAN;
    row.pressurePa = s.fields & telemetry::Pressure ? s.pressurePa : NAN;
    row.altitudeM = s.fields & telemetry::Altitude ? s.altitudeM : NAN;
    row.relativeM = row.tiltDeg = NAN;
    row.baselineM = monitor.zeroed() ? monitor.baselineM() : NAN;
    if (monitor.relativeHeight(now, cfg, row.relativeM)) row.validity |= 2;
    if (monitor.tilt(now, cfg, row.tiltDeg)) row.validity |= 4;
    buffer_[count_++] = row; ++rows_;
    if (count_ == 8 && !flush()) stop("WRITE ERROR", now);
}

void Recorder::stop(const char* reason, uint32_t now) {
    if (!active_) return;
    if (!flush()) reason = "WRITE ERROR";
    file_.flush(); file_.close(); count_ = 0; active_ = false; status_ = reason;
    events_.flush(); events_.close();
    File end = LittleFS.open("/end.json", "w");
    if (end) {
        end.printf("{\"end_ms\":%lu,\"accepted_rows\":%lu,\"reason\":\"%s\"}\n",
                   (unsigned long)now, (unsigned long)rows_, reason);
        end.close();
    }
}

bool Recorder::startExport(const char* name) {
    if (!mounted_ || active_ || exportActive_) return false;
    exportCsv_ = strcmp(name, "CSV") == 0;
    const char* path = exportCsv_ ? "/samples.bin" :
        (strcmp(name, "META") == 0 ? "/session.json" :
         (strcmp(name, "END") == 0 ? "/end.json" :
          (strcmp(name, "EVENTS") == 0 ? "/events.csv" : nullptr)));
    if (!path) return false;
    export_ = LittleFS.open(path, "r");
    if (!export_) return false;
    if (exportCsv_) {
        char magic[8];
        if (export_.readBytes(magic, 8) != 8 || memcmp(magic, header, 8) != 0) {
            export_.close(); return false;
        }
    }
    txAt_ = 0;
    txSize_ = snprintf(tx_, sizeof(tx_), "BEGIN %s\n%s", name, exportCsv_ ? csvHeader : "");
    exportActive_ = true; exportEnding_ = false;
    return true;
}

void Recorder::cancelExport() { export_.close(); txAt_ = txSize_ = 0; exportActive_ = exportEnding_ = false; }

void Recorder::tick(uint32_t now) {
    if (active_ && uint32_t(now - started_) >= limitMs_) stop("TIME LIMIT", now);
    if (active_ && uint32_t(now - lastFlush_) >= 500) {
        if (!flush()) stop("WRITE ERROR", now);
        else { file_.flush(); events_.flush(); lastFlush_ = now; }
    }
    if (!exportActive_) return;
    if (!Serial) { cancelExport(); return; }
    if (txAt_ == txSize_) {
        txAt_ = 0;
        if (exportEnding_) { cancelExport(); return; }
        if (!export_.available()) {
            export_.close();
            txSize_ = snprintf(tx_, sizeof(tx_), "END\n"); exportEnding_ = true;
        } else if (!exportCsv_) txSize_ = export_.read(reinterpret_cast<uint8_t*>(tx_), sizeof(tx_));
        else {
            LogRow row;
            if (export_.read(reinterpret_cast<uint8_t*>(&row), sizeof(row)) != sizeof(row)) {
                export_.close();
                txSize_ = snprintf(tx_, sizeof(tx_), "ERR TRUNCATED_LOG\n"); exportEnding_ = true;
            } else {
                txSize_ = snprintf(tx_, sizeof(tx_), "%lu,%u,%u,%u,%u",
                    (unsigned long)row.timeMs, row.fields, row.type, row.magneticQuality, row.validity);
                float values[22];
                memcpy(values, &row.accel[0], sizeof(values));
                for (float v : values) {
                    txSize_ += snprintf(tx_ + txSize_, sizeof(tx_) - txSize_, ",%.6f", double(v));
                }
                tx_[txSize_++] = '\n';
            }
        }
    }
    const size_t capacity = Serial.availableForWrite();
    const size_t remaining = txSize_ - txAt_;
    const size_t n = capacity < remaining ? capacity : remaining;
    if (n) txAt_ += Serial.write(reinterpret_cast<uint8_t*>(tx_ + txAt_), n);
}
