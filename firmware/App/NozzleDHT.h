#ifndef NOZZLE_DHT_H
#define NOZZLE_DHT_H

#include "Constants.h"

#ifdef Nozzle_Mount_PCB
void setupNozzleDHT();
void serviceNozzleDHT();
void reportNozzleDHT(Print& output);
bool diagnosticNozzleDHT();
const char* diagnosticNozzleDHTCode();
#endif

#endif
