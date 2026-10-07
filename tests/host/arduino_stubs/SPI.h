#pragma once
#include "Arduino.h"
#define SPI_MODE0 0
#define SPI_MODE3 3
struct SPISettings { SPISettings(uint32_t = 0, uint8_t = 0, uint8_t = 0) {} };
class SPIClass {
public:
    void begin() {}
    void beginTransaction(SPISettings) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t) { return 0; }
    uint16_t transfer16(uint16_t) { return 0; }
};
extern SPIClass SPI;
