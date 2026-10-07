/*
 * MagazineLED.h
 *
 * Local NeoPixel status display for Magazine Holder PCB V2.
 * Each holder owns one 44-pixel strip on PE2.
 */

#ifndef MAGAZINELED_H_
#define MAGAZINELED_H_

#include "Constants.h"

#if defined(Magazine_PCB_V2)

#include <Adafruit_NeoPixel.h>
#include "Magazine_PCB_V2.h"

class MagazineLED {
public:
    MagazineLED();

    void setup();
    void setThresholds(uint8_t magazineType, const uint16_t values[20]);
    uint8_t getThresholdValidMask() const;
    bool classifySlots(uint8_t magazineType, const int values[20], uint8_t slots[20]) const;
    void loop(const int* irValues, bool valuesReady);
    void off();
    bool hasPendingShow() const;
    void show();

private:
    static constexpr uint16_t LED_COUNT = 44;
    static constexpr uint8_t SLOT_COUNT = 20;
    static constexpr uint8_t MAX_MAG_TYPES = 7;
    // Fast local feedback: no Gantry transaction is required to render insertion.
    static constexpr uint16_t REFRESH_MS = 20;
    static constexpr uint16_t BLINK_MS = 200;
    static constexpr uint8_t LOGO_DIM = 50;
    static constexpr uint8_t LEARN_SAMPLES = 10;
    // Two 20 ms confirmations reject a one-sample spike without visible lag.
    static constexpr uint8_t STATE_DEBOUNCE_SAMPLES = 2;
    static constexpr uint16_t STATE_HYSTERESIS = 20;
    static constexpr uint16_t EMPTY_TO_PRESENT_DELTA = 200;

    Adafruit_NeoPixel strip;
    uint16_t thresholds[MAX_MAG_TYPES][SLOT_COUNT] = {};
    uint8_t thresholdValidMask = 0;
    uint8_t learningType = 0;
    uint8_t learningSamples = 0;
    uint32_t learningSums[SLOT_COUNT] = {};
    bool slotPresent[SLOT_COUNT] = {};
    uint8_t slotDebounce[SLOT_COUNT] = {};
    uint8_t lastMagazineType = 0;
    bool showPending = false;

    unsigned long lastBlinkMs = 0;
    unsigned long lastRefreshMs = 0;
    bool blinkState = false;

    int detectMagazineType(const int* irValues) const;
    bool hasThresholds(uint8_t magazineType) const;
    bool isMagazineFull() const;
    void resetSlotState();
    void resetLearning(uint8_t magazineType);
    void learnThresholds(uint8_t magazineType, const int* irValues);
    void updateSlotState(const int* irValues, const uint16_t* activeThresholds);
    bool setPixel(uint16_t index, uint32_t color);
    bool setLogo(bool magazinePresent);
};

extern MagazineLED magazineLED;

#endif
#endif

