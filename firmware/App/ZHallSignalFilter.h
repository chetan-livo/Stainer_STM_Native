#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// Task-context filter only. Raw ADC packets must remain unmodified.
// Median rejects short impulses; the time-based pole tracks sustained changes.
// Defaults: 200 Hz acquisition, median-5 (10 ms delay), tau=20 ms.
namespace ZHallSignal {
template<size_t Window=5> class Filter {
    static_assert(Window>=3 && Window<=5 && (Window&1),"Use median 3 or 5");
    int samples[10][Window] = {};
    float values[10] = {};
    size_t count=0,index=0;
    uint32_t lastMs=0;
    bool initialized=false;
    float tauMs;
    uint32_t gapMs;
public:
    explicit Filter(float tau=20.0f,uint32_t gap=50):tauMs(tau),gapMs(gap){}
    void reset(){count=0;index=0;initialized=false;}
    const float* output() const{return values;}
    bool update(const int* raw,uint32_t now) {
        for(size_t i=0;i<10;++i)if(raw[i]<0||raw[i]>4095){reset();return false;}
        const uint32_t elapsed=now-lastMs;
        if(count && elapsed>=gapMs)reset();
        lastMs=now;
        for(size_t i=0;i<10;++i)samples[i][index]=raw[i];
        index=(index+1)%Window;
        if(count<Window)++count;
        if(count<Window)return false;
        const float alpha= tauMs>0 ? elapsed/(tauMs+elapsed) : 1.0f;
        for(size_t i=0;i<10;++i){
            int sorted[Window];
            for(size_t j=0;j<Window;++j)sorted[j]=samples[i][j];
            for(size_t j=1;j<Window;++j){
                const int v=sorted[j];size_t k=j;
                while(k && sorted[k-1]>v){sorted[k]=sorted[k-1];--k;}
                sorted[k]=v;
            }
            const float median=sorted[Window/2];
            values[i]=initialized ? values[i]+alpha*(median-values[i]) : median;
        }
        initialized=true;return true;
    }
};
}
