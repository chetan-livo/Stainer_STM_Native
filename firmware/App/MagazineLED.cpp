/*
 * MagazineLED.cpp
 */

#include "MagazineLED.h"
#include "SensorFaultPolicy.h"

#if defined(Magazine_PCB_V2)

#include <string.h>

MagazineLED magazineLED;

MagazineLED::MagazineLED()
    : strip(LED_COUNT, MAG_LED_DATA, NEO_GRB + NEO_KHZ800) {}

void MagazineLED::setup() {
    strip.begin();
    strip.setBrightness(255);
    strip.clear();
    for (uint8_t i = 0; i < 4; ++i) {
        strip.setPixelColor(i, strip.Color(LOGO_DIM, LOGO_DIM, LOGO_DIM));
    }
    strip.show();
    showPending = false;
}

void MagazineLED::setThresholds(uint8_t magazineType, const uint16_t values[SLOT_COUNT]) {
    if (!values || magazineType < 1 || magazineType > MAX_MAG_TYPES) return;
    memcpy(thresholds[magazineType - 1], values, sizeof(thresholds[0]));
    thresholdValidMask |= (1U << (magazineType - 1));
    if (learningType == magazineType) {
        learningType = 0;
        learningSamples = 0;
    }
    resetSlotState();
    lastRefreshMs = 0;
}

int MagazineLED::detectMagazineType(const int* irValues) const {
    if (!irValues) return 0;
    for(int i=20;i<23;++i) if(!SensorFaultPolicy::adcValid(irValues[i])) return 0;
    const int s21 = irValues[20] < MAG_TYPE_S21_THRESHOLD ? 1 : 0;
    const int s22 = irValues[21] < MAG_TYPE_S22_THRESHOLD ? 1 : 0;
    const int s23 = irValues[22] < MAG_TYPE_S23_THRESHOLD ? 1 : 0;
    return (s23 << 2) | (s22 << 1) | s21;
}

bool MagazineLED::hasThresholds(uint8_t magazineType) const {
    if (magazineType < 1 || magazineType > MAX_MAG_TYPES) return false;
    return (thresholdValidMask & (1U << (magazineType - 1))) != 0;
}

uint8_t MagazineLED::getThresholdValidMask() const {
    return thresholdValidMask;
}

bool MagazineLED::classifySlots(uint8_t magazineType, const int values[SLOT_COUNT],
                                uint8_t slots[SLOT_COUNT]) const {
    if (!values || !slots || !hasThresholds(magazineType)) return false;
    const uint16_t* activeThresholds = thresholds[magazineType - 1];
    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
        slots[slot] = !SensorFaultPolicy::adcValid(values[slot]) ? 3 : values[slot] < activeThresholds[slot] ? 1 : 0;
    }
    return true;
}

bool MagazineLED::isMagazineFull() const {
    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
        if (!slotPresent[slot]) return false;
    }
    return true;
}

void MagazineLED::resetSlotState() {
    memset(slotPresent, 0, sizeof(slotPresent));
    memset(slotDebounce, 0, sizeof(slotDebounce));
}

void MagazineLED::resetLearning(uint8_t magazineType) {
    learningType = magazineType;
    learningSamples = 0;
    memset(learningSums, 0, sizeof(learningSums));
    resetSlotState();
}

void MagazineLED::learnThresholds(uint8_t magazineType, const int* irValues) {
    if (!irValues || magazineType < 1 || magazineType > MAX_MAG_TYPES) return;
    for(uint8_t slot=0;slot<SLOT_COUNT;++slot) if(!SensorFaultPolicy::adcValid(irValues[slot])) { resetLearning(magazineType);return; }
    if (learningType != magazineType) resetLearning(magazineType);

    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
        learningSums[slot] += (uint16_t)constrain(irValues[slot], 0, 65535);
    }

    if (++learningSamples < LEARN_SAMPLES) return;

    uint16_t* learned = thresholds[magazineType - 1];
    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
        const uint32_t emptyReference = learningSums[slot] / LEARN_SAMPLES;
        learned[slot] = (uint16_t)(emptyReference > EMPTY_TO_PRESENT_DELTA
            ? emptyReference - EMPTY_TO_PRESENT_DELTA
            : 0);
    }
    thresholdValidMask |= (1U << (magazineType - 1));
    learningType = 0;
    learningSamples = 0;
    resetSlotState();
}

void MagazineLED::updateSlotState(const int* irValues, const uint16_t* activeThresholds) {
    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
        if(!SensorFaultPolicy::adcValid(irValues[slot])) {slotDebounce[slot]=0;continue;}
        const int threshold = activeThresholds[slot];
        const bool candidate = slotPresent[slot]
            ? irValues[slot] < threshold + STATE_HYSTERESIS
            : irValues[slot] < threshold - STATE_HYSTERESIS;

        if (candidate == slotPresent[slot]) {
            slotDebounce[slot] = 0;
        } else if (++slotDebounce[slot] >= STATE_DEBOUNCE_SAMPLES) {
            slotPresent[slot] = candidate;
            slotDebounce[slot] = 0;
        }
    }
}

bool MagazineLED::setPixel(uint16_t index, uint32_t color) {
    if (strip.getPixelColor(index) == color) return false;
    strip.setPixelColor(index, color);
    return true;
}

bool MagazineLED::setLogo(bool magazinePresent) {
    const uint32_t color = magazinePresent
        ? strip.Color(LOGO_DIM, LOGO_DIM, LOGO_DIM)
        : strip.Color(0, 0, 0);
    bool changed = false;
    for (uint8_t i = 0; i < 4; ++i) changed |= setPixel(i, color);
    return changed;
}

void MagazineLED::loop(const int* irValues, bool valuesReady) {
    const unsigned long now = millis();
    bool blinkChanged = false;
    if (now - lastBlinkMs >= BLINK_MS) {
        lastBlinkMs = now;
        blinkState = !blinkState;
        blinkChanged = true;
    }

    if (!valuesReady || !irValues || now - lastRefreshMs < REFRESH_MS) {
        if (blinkChanged && lastMagazineType != 0 && hasThresholds(lastMagazineType) && isMagazineFull()) {
            showPending = true;
            const uint32_t color = blinkState ? strip.Color(255, 255, 255) : strip.Color(0, 0, 0);
            for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
                const uint16_t firstLed = 4 + slot * 2;
                setPixel(firstLed, color);
                setPixel(firstLed + 1, color);
            }
        }
        return;
    }
    lastRefreshMs = now;

    const uint8_t magazineType = (uint8_t)detectMagazineType(irValues);
    if (magazineType != lastMagazineType) {
        lastMagazineType = magazineType;
        resetSlotState();
        if (magazineType != 0 && !hasThresholds(magazineType)) resetLearning(magazineType);
    }

    const uint16_t* activeThresholds = nullptr;
    if (magazineType != 0) {
        if (hasThresholds(magazineType)) {
            activeThresholds = thresholds[magazineType - 1];
            updateSlotState(irValues, activeThresholds);
        }
    }

    bool changed = setLogo(magazineType != 0);
    const bool full = activeThresholds && isMagazineFull();

    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
        uint32_t color;
        if (magazineType == 0) {
            color = strip.Color(0, 0, 0);
        } else if (full) {
            color = blinkState ? strip.Color(255, 255, 255) : strip.Color(0, 0, 0);
        } else if (!activeThresholds) {
            color = strip.Color(0, 0, 50);
        } else if (slotPresent[slot]) {
            color = strip.Color(0, 200, 0);
        } else {
            color = strip.Color(200, 0, 0);
        }

        const uint16_t firstLed = 4 + slot * 2;
        changed |= setPixel(firstLed, color);
        changed |= setPixel(firstLed + 1, color);
    }

    if (changed || (full && blinkChanged)) showPending = true;
}

void MagazineLED::off() {
    strip.clear();
    showPending = true;
}

bool MagazineLED::hasPendingShow() const {
    return showPending;
}

void MagazineLED::show() {
    if (!showPending) return;
    strip.show();
    showPending = false;
}

#endif

