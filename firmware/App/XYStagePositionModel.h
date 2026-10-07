/*
* XYStagePositionModel.h
*
*  Created on: 22-Apr-2025
*      Author: Varalakshmi
*/

#ifndef XYSTAGEPOSITION_MODEL_H
#define XYSTAGEPOSITION_MODEL_H

#include <Arduino.h>
#include <math.h>

class XYStagePositionModel {
    public:
        static constexpr uint8_t NUM_FEATURES = 10;
        static int32_t predictFromRaw(const int raw[]);

        XYStagePositionModel();
        virtual ~XYStagePositionModel();
    };

#endif