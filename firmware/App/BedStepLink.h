#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "BedStepScheduler.h"

namespace BedSteps {
// All messages carry a run epoch, physical STEP coordinate and checksum.
// R=establish epoch, T=position barrier, A=barrier acknowledged, Q=action,
// F=latched fault, P=pause. No position is extrapolated from a clock.
struct Frame { char kind; uint32_t epoch, step, value; };
inline uint32_t checksum(const Frame& f) {
    uint32_t h = (2166136261UL ^ (uint8_t)f.kind) * 16777619UL;
    const uint32_t fields[] = {f.epoch, f.step, f.value};
    for (auto v : fields) for (unsigned i=0;i<4;++i) { h=(h^(v&255))*16777619UL; v>>=8; }
    return h;
}
inline bool parse(const char* line, Frame& f) {
    unsigned long e,p,v,c; char kind,extra;
    if (sscanf(line,"BS2 %c %lx %lx %lx %lx %c",&kind,&e,&p,&v,&c,&extra)!=5 ||
        !strchr("RTAQFPC",kind) || e>UINT32_MAX || p>UINT32_MAX || v>UINT32_MAX || c>UINT32_MAX) return false;
    f={kind,(uint32_t)e,(uint32_t)p,(uint32_t)v};
    return checksum(f)==(uint32_t)c;
}
template<class Port> inline void send(Port& port, char kind, uint32_t epoch, uint32_t step, uint32_t value=0) {
    Frame f={kind,epoch,step,value}; char line[64];
    snprintf(line,sizeof(line),"BS2 %c %lX %lX %lX %lX\n",kind,
        (unsigned long)epoch,(unsigned long)step,(unsigned long)value,(unsigned long)checksum(f));
    port.print(line);
}
// Wire action IDs intentionally follow the absolute recipe columns (k + 1).
// BS2 prevents mixing this schema with older relative-cascade firmware.
// Buffer1 avoids Arduino's legacy B1 binary-literal macro. Wire ID stays 6.
enum Action : uint16_t { EthPump=1, EthOn, EthOff, SY, S1, Buffer1, AirOn, AirOff,
    SX, Wash1On, Wash1Off, Suction1On, Suction1Off, S2Start, S2Dispatch,
    Wash2On, Wash2Off, Suction2On, Suction2Off, WY, WX, DryOn, DryOff, ActionCount };
constexpr uint32_t LinkTimeoutMs=1000; // watchdog only; never a production deadline
}
