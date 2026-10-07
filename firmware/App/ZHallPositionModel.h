#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <float.h>

// Step-equivalent Hall fingerprint interpolation. Tables must be validated on
// separate measurements before being enabled. No commanded-position input is
// used to choose a segment: that would conceal a missed-step error.
namespace ZHallPosition {
struct Knot { int32_t steps; float adc[10]; };
struct Table {
    const Knot* knots;
    size_t count;
    const float* inverseVariance;
    float residualLimit;
    float sigmaFloorSteps;
    float maxSigmaSteps;
    bool validated;
};
enum Status : uint8_t { Uncalibrated, Valid, InvalidAdc, WeakSignal, OffCurve, Ambiguous, OutsideRange };
struct Result {
    Status status = Uncalibrated;
    float steps = 0;
    float sigmaSteps = 0;
    float residual = 0;
};
struct Projection { float position, sigma, residual, fraction; bool usable; };
inline Projection project(const float* adc, const Table& table, size_t segment) {
    const Knot& a=table.knots[segment]; const Knot& b=table.knots[segment+1];
    float norm=0, dot=0;
    for (size_t i=0;i<10;++i) {
        const float delta=b.adc[i]-a.adc[i];
        norm+=table.inverseVariance[i]*delta*delta;
        dot+=table.inverseVariance[i]*(adc[i]-a.adc[i])*delta;
    }
    if (!(norm>0) || b.steps<=a.steps) return {0,0,FLT_MAX,0,false};
    const float fraction=dot/norm;
    const float clamped=fminf(1,fmaxf(0,fraction));
    float residual=0;
    for (size_t i=0;i<10;++i) {
        const float error=adc[i]-a.adc[i]-clamped*(b.adc[i]-a.adc[i]);
        residual+=table.inverseVariance[i]*error*error;
    }
    const float span=(float)(b.steps-a.steps);
    return {a.steps+clamped*span,fmaxf(table.sigmaFloorSteps,span/sqrtf(norm)),residual,fraction,true};
}
inline Result predict(const float* adc,const Table& table) {
    Result result;
    if (!table.validated || !table.knots || !table.inverseVariance || table.count<2) return result;
    for (size_t i=0;i<10;++i) {
        if (!isfinite(adc[i]) || adc[i]<0 || adc[i]>4095) {result.status=InvalidAdc;return result;}
    }
    Projection best={0,0,FLT_MAX,0,false}; size_t bestSegment=0;
    for (size_t s=0;s+1<table.count;++s) {
        const Projection candidate=project(adc,table,s);
        if (candidate.usable && candidate.residual<best.residual) {best=candidate;bestSegment=s;}
    }
    if (!best.usable || best.sigma>table.maxSigmaSteps) {result.status=WeakSignal;return result;}
    result.steps=best.position;result.sigmaSteps=best.sigma;result.residual=best.residual;
    if (best.residual>table.residualLimit) {result.status=OffCurve;return result;}
    if ((bestSegment==0 && best.fraction<0) || (bestSegment+2==table.count && best.fraction>1)) {
        result.status=OutsideRange;return result;
    }
    for (size_t s=0;s+1<table.count;++s) {
        if (s==bestSegment) continue;
        const Projection other=project(adc,table,s);
        if (other.usable && other.residual<=best.residual+9 &&
            fabsf(other.position-best.position)>fmaxf(100,3*best.sigma)) {
            result.status=Ambiguous;return result;
        }
    }
    result.status=Valid;return result;
}
}
