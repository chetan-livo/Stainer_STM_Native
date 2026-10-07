#pragma once
#include <Arduino.h>
namespace ServiceDiagnostics {
bool locked();
bool consume(const String& command);
void loop();
void result(const char* code,const char* status,const String& evidence);
}
void serviceDiagnosticQuiesce();
void serviceDiagnosticRelease();
void serviceDiagnosticPeripherals();
// Production has priority over discovery and intrusive service exercises.
bool serviceProductionBusy();
bool serviceBackgroundCommand(const String& command);
