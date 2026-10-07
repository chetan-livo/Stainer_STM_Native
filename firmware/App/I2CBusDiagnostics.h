#pragma once
#include <Arduino.h>
#include <Wire.h>

// Read-only diagnostics for the STM32F4 buses. Do not read SR1 followed by
// SR2 here: that sequence can acknowledge/clear a live slave ADDR event.
inline void printI2cBusDiagnostics(TwoWire& wire, Print& out, const char* label,
                                   uint8_t sda, uint8_t scl) {
    auto* handle = wire.getHandle();
    out.print("I2CSTAT "); out.print(label);
    out.print(" SDA="); out.print(digitalRead(sda));
    out.print(" SCL="); out.print(digitalRead(scl));
    out.print(" STATE=0x"); out.print((uint32_t)handle->State, HEX);
    out.print(" ERROR=0x"); out.print((uint32_t)handle->ErrorCode, HEX);
    if (handle->Instance) {
        const uint32_t cr1 = handle->Instance->CR1;
        const uint32_t oar1 = handle->Instance->OAR1;
        out.print(" PE="); out.print((cr1 & I2C_CR1_PE) ? 1 : 0);
        out.print(" ACK="); out.print((cr1 & I2C_CR1_ACK) ? 1 : 0);
        out.print(" OWN7=0x"); out.print((oar1 >> 1) & 0x7F, HEX);
    } else {
        out.print(" PERIPHERAL=UNINITIALIZED");
    }
    out.println();
}
