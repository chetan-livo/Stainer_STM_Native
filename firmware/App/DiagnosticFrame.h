#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace DiagnosticFrame {
inline uint32_t crc(const char* data,size_t size) {
    uint32_t value=0xffffffffu;
    while(size--) {value^=(uint8_t)*data++;for(unsigned b=0;b<8;++b)value=(value>>1)^(0xedb88320u & (0u-(value&1u)));}
    return ~value;
}
// Verify the entire payload, including session, board, sequence and status.
inline bool valid(const char* frame,size_t size,size_t& payload) {
    if(size<13)return false;
    payload=size-13;
    if(memcmp(frame+payload," CRC=",5))return false;
    uint32_t expected=0;
    for(size_t i=payload+5;i<size;++i) {
        char c=frame[i];unsigned n=c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:c>='A'&&c<='F'?c-'A'+10:16;
        if(n>15)return false;expected=(expected<<4)|n;
    }
    return expected==crc(frame,payload);
}
}
