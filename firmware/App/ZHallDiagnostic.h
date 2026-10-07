#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>

// One immutable, paired raw/filtered snapshot per USB request. Formatting uses
// integer milli-ADC units, so newlib float printf support is not required.
namespace ZHallDiagnostic {
inline size_t format(char* out,size_t capacity,uint32_t sequence,uint32_t sampleMs,
                     uint32_t age,bool ready,const int* raw,const float* filtered) {
    if(!capacity)return 0;
    int n=snprintf(out,capacity,"\nZHFILT,%lu,%lu,%lu,%u",(unsigned long)sequence,
                   (unsigned long)sampleMs,(unsigned long)age,ready?1:0);
    if(n<0||(size_t)n>=capacity)return 0;
    size_t used=n;
    for(size_t i=0;i<20;++i){
        const long value=i<10 ? raw[i] : (ready ? lroundf(filtered[i-10]*1000) : -1);
        n=snprintf(out+used,capacity-used,",%ld",value);
        if(n<0||(size_t)n>=capacity-used)return 0;
        used+=n;
    }
    if(used+2>capacity)return 0;
    out[used++]='\n';out[used]=0;return used;
}
}
