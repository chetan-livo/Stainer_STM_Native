/*
 * LidarReader.h
 *
 * Created on: Apr 2, 2026
 * Author: OpenAI Codex
 */

#ifndef LIDARREADER_H_
#define LIDARREADER_H_

#include "Constants.h"

#ifdef Master
    #ifdef Stainer_Gantry_PCB
        #include <Arduino.h>
        #include <Wire.h>
        #include <VL53L0X.h>

        class LidarReader {
        public:
            explicit LidarReader(TwoWire& wire);

            void setup();
            bool isReady() const { return initialized_; }

            float correctDistance(uint16_t raw) const;
            uint16_t readAverageRaw(uint8_t count = 10);
            bool readCorrectedDistanceMm(float& correctedDistanceMm, uint8_t count = 10);
            bool readGxPositionMm(float& gxPositionMm, uint8_t count = 10);

        private:
            TwoWire& wire_;
            VL53L0X sensor_;
            bool initialized_ = false;
        };

        extern LidarReader lidarReader;
    #endif
#endif

#endif /* LIDARREADER_H_ */
