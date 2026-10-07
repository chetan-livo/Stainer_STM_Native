#pragma once
#include "Constants.h"
#include "SensorFaultPolicy.h"
namespace SensorFault {
// Task/loop context only. Never print from an interrupt or transport callback.
void report(const char* code, const char* sensor, long raw, const char* reason);
}
