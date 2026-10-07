#include "NozzleDHT.h"
#include "SensorFault.h"
#include "ServiceDiagnostics.h"

#ifdef Nozzle_Mount_PCB
#include "NozzleMountPCBV1.h"
#include "DCFan.h"
#include "src/AdafruitDHT/DHT.h"
#include <math.h>

namespace {
DHT nozzleDHT(IR9, DHT11);
uint32_t lastAttemptAt = 0;
constexpr uint32_t sampleIntervalMs = 2500;
bool attemptedRead = false;
bool readingValid = false;
float humidity = NAN;
float temperature = NAN;
int fan1Pwm = 0;

// Stainer180 stores humidity as an integer before calculating fan PWM.
constexpr int humidityFanPwm(int rh) {
    return rh <= 35 ? 150 : (rh >= 60 ? 255 : (42 * rh + 30) / 10);
}
static_assert(humidityFanPwm(35) == 150 && humidityFanPwm(36) == 154,
              "Humidity lower boundary must match Stainer180");
static_assert(humidityFanPwm(50) == 213 && humidityFanPwm(59) == 250 &&
              humidityFanPwm(60) == 255, "Humidity PWM mapping regression");

void applyFan1Pwm(int pwm) {
    if(ServiceDiagnostics::locked())return;
    fan1Pwm = pwm;
    dcFan1.runFan(pwm);
}
}

void setupNozzleDHT() {
    nozzleDHT.begin();
    lastAttemptAt = millis();
    attemptedRead = false;
    readingValid = false;
    applyFan1Pwm(0);
}

void serviceNozzleDHT() {
    // DHT11 uses delay(20) and an interrupt-masked pulse read. Keep the last
    // fan setting during production; refresh environmental data when idle.
    if (serviceProductionBusy()) return;
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - lastAttemptAt) < sampleIntervalMs) return;
    lastAttemptAt = now;
    attemptedRead = true;
    // Both fields use the same checksum-validated transaction in the driver.
    humidity = nozzleDHT.readHumidity();
    temperature = nozzleDHT.readTemperature();
    readingValid = isfinite(humidity) && isfinite(temperature) &&
                   humidity >= 0.0f && humidity <= 100.0f && temperature >= 0.0f && temperature <= 50.0f;
    if (readingValid) {
        applyFan1Pwm(humidityFanPwm(static_cast<int>(humidity)));
    }
    else {
        const char* reason=isfinite(humidity)&&isfinite(temperature)?"physical_range_invalid":nozzleDHT.lastError();
        const char* code="LIVO-SEN-012";
        if (!strcmp(reason,"RESPONSE_LOW_TIMEOUT")) code="LIVO-SEN-013";
        else if (!strcmp(reason,"RESPONSE_HIGH_TIMEOUT")) code="LIVO-SEN-014";
        else if (!strcmp(reason,"DATA_PULSE_TIMEOUT")) code="LIVO-SEN-015";
        else if (strstr(reason,"CHECKSUM")) code="LIVO-SEN-016";
        SensorFault::report(code,"DHT11",digitalRead(IR9),reason);
    }
    // An invalid sample cannot replace the last valid fan command.
}

void reportNozzleDHT(Print& output) {
    serviceNozzleDHT();
    if (!attemptedRead) {
        output.println("RDHT ERROR:WARMUP RETRY_AFTER_MS:2500");
        return;
    }
    if (!readingValid) {
        output.print("RDHT ERROR:READ_FAILED REASON:");
        output.print(isfinite(humidity) && isfinite(temperature) ?
                     "PHYSICAL_RANGE_INVALID" : nozzleDHT.lastError());
        output.print(" PIN:IR9/PB1 DATA_LEVEL:");
        output.print(digitalRead(IR9));
        output.print(" DCF1_PWM:");
        output.print(fan1Pwm);
        output.print(" DCF1_MODE:");
        output.print("AUTO");
        output.println(" RETRY_AFTER_MS:2500");
        return;
    }
    output.print("RDHT TEMP_C:");
    output.print(temperature, 1);
    output.print(" HUMIDITY_PCT:");
    output.print(humidity, 1);
    output.print(" DCF1_PWM:");
    output.print(fan1Pwm);
    output.print(" DCF1_MODE:");
    output.println("AUTO");
}
bool diagnosticNozzleDHT(){serviceNozzleDHT();return attemptedRead && readingValid && millis()-lastAttemptAt<3000;}
const char* diagnosticNozzleDHTCode(){
    if(readingValid)return "LIVO-SEN-012";
    const char* reason=nozzleDHT.lastError();
    if(!strcmp(reason,"RESPONSE_LOW_TIMEOUT"))return "LIVO-SEN-013";
    if(!strcmp(reason,"RESPONSE_HIGH_TIMEOUT"))return "LIVO-SEN-014";
    if(!strcmp(reason,"DATA_PULSE_TIMEOUT"))return "LIVO-SEN-015";
    if(strstr(reason,"CHECKSUM"))return "LIVO-SEN-016";
    return "LIVO-SEN-012";
}
#endif
