Vendored core of Adafruit DHT sensor library 1.4.6 (MIT).

Source: https://github.com/adafruit/DHT-sensor-library/tree/1.4.6

Only DHT.cpp, DHT.h and license.txt are included. The optional unified-sensor
wrapper is not used, so no Adafruit Unified Sensor dependency is required.

Local extension: `lastError()` reports response-low timeout, response-high timeout,
data-pulse timeout or checksum failure. It preserves the error when a cached
read result is returned and does not print inside the timing-critical capture.
