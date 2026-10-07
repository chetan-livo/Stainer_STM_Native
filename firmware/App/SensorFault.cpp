#include "SensorFault.h"
#include <string.h>
namespace SensorFault {
void report(const char* code, const char* sensor, long raw, const char* reason) {
#ifndef Master
    (void)code; (void)sensor; (void)raw; (void)reason;
    return; // Holder errors are emitted by the receiving controller with holder identity.
#else
    struct Recent { uint32_t key=0, at=0; bool used=false; };
    static Recent recent[32]; static uint8_t next=0;
    uint32_t key=2166136261UL;
    for (const char* p=code; *p; ++p) key=(key^(uint8_t)*p)*16777619UL;
    for (const char* p=sensor; *p; ++p) key=(key^(uint8_t)*p)*16777619UL;
    const uint32_t now=millis();
    for (auto& item:recent) if(item.used && item.key==key) {
        if((uint32_t)(now-item.at)<60000UL)return;
        item.at=now; goto emit;
    }
    recent[next].used=true; recent[next].key=key; recent[next].at=now;
    next=(next+1)%32;
emit:
#ifdef Master
    Print& output=mirroredSerial;
#else
    Print& output=Serial;
#endif
    output.print("SFAULT CODE="); output.print(code);
    output.print(" BOARD="); output.print(DEVICE_ID);
    output.print(" SENSOR="); output.print(sensor);
    output.print(" RAW="); output.print(raw);
    output.print(" REASON="); output.println(reason);
#endif
}
}
