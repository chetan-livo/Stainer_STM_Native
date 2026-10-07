#pragma once
#include <stdint.h>
#include <stddef.h>

// Same 32-sample mean as the original block estimator, published every 4 samples.
// Actual sample timestamps describe the window; consumers need not assume 200 Hz.
namespace ZHallSignal {
class PositionWindow {
    float samples[32][10] = {};
    uint32_t times[32] = {};
    uint8_t count=0, next=0, sincePublish=0;
public:
    void reset() { count=0; next=0; sincePublish=0; }
    bool update(const float* values, uint32_t now, float* mean, uint32_t& span) {
        for(size_t i=0;i<10;++i) samples[next][i]=values[i];
        times[next]=now; next=(next+1)%32;
        if(count<32) ++count;
        if(++sincePublish<4 || count<32) return false;
        sincePublish=0;
        // Sum afresh to avoid accumulated float rounding drift over long runs.
        for(size_t i=0;i<10;++i) {
            float total=0;
            for(size_t j=0;j<32;++j) total+=samples[j][i];
            mean[i]=total/32;
        }
        span=now-times[next];
        return true;
    }
};
}
