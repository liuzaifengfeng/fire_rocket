#include "TelemetryCore.h"
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <cmath>
#include <limits>

// Input: monotonic milliseconds followed by hexadecimal bytes, TICK or ZERO.
// This adapter runs the actual firmware parser and monitor; it has no GPIO code.
int main() {
    telemetry::Parser parser; telemetry::Monitor monitor; telemetry::Settings cfg;
    std::string line; uint32_t revision=0, previous=0; bool first=true;
    std::cout << "t_ms,event,samples,fresh,zeroed,altitude_m,relative_m,tilt_deg,checksum_errors,framing_errors,data_errors\n";
    while (std::getline(std::cin,line)) {
        if (line.empty() || line[0]=='#') continue;
        std::istringstream row(line); uint64_t time; std::string hex,extra;
        if (!(row>>time>>hex) || (row>>extra) || time>0xffffffffu || (!first&&time<previous)) return 2;
        first=false; previous=uint32_t(time); const uint32_t now=uint32_t(time);
        std::string event="DATA";
        if (hex=="ZERO") event=monitor.zero(now,cfg)?"ZERO_OK":"ZERO_REFUSED";
        else if (hex=="TICK") event="TICK";
        else {
            if (hex.size()%2) return 2;
            for (size_t i=0;i<hex.size();i+=2) {
                std::string pair=hex.substr(i,2); char* end=nullptr;
                unsigned long value=std::strtoul(pair.c_str(),&end,16);
                if (*end || value>255) return 2;
                bool updated=parser.feed(uint8_t(value),now);
                if (parser.configRevision()!=revision) { revision=parser.configRevision(); monitor.invalidate(); }
                if (updated) monitor.accept(parser.sample(),cfg);
            }
        }
        float h=std::numeric_limits<float>::quiet_NaN(),tilt=h,alt=h;
        monitor.relativeHeight(now,cfg,h); monitor.tilt(now,cfg,tilt);
        if (monitor.fresh(now,cfg)&&(monitor.sample().fields&telemetry::Altitude)) alt=monitor.sample().altitudeM;
        const auto& counters=parser.counters();
        std::cout << now << ',' << event << ',' << counters.samples << ',' << monitor.fresh(now,cfg) << ','
            << monitor.zeroed() << ',' << alt << ',' << h << ',' << tilt << ',' << counters.checksumErrors << ','
            << counters.framingErrors << ',' << counters.dataErrors << '\n';
    }
    return 0;
}
