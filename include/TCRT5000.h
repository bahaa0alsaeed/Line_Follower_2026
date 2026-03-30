#pragma once
#include "pins_validation.h"
#include <Arduino.h>
#include <array>
using namespace std;

// TODO Add a separate threshold for each sensor.
// TODO Add automatic calibration and normalization.
template<
    uint8_t readPin, uint8_t ledOn, uint8_t s0, uint8_t s1,
    uint8_t s2, uint32_t threshold, uint8_t numSensors = 8
>
class TCRT5000 {
    float lastError = 0.0f;

public:
    // ------------ Compile-Time Safety ------------
    static constexpr bool isValidMapping() {
        // 1. Verify that all pins belong to the allowed ESP32 pin list
        if (!isPinValid(readPin, ADC | Input) || !isPinValid(ledOn, Output | PWM) ||
            !isPinValid(s0, Output) || !isPinValid(s1, Output) || !isPinValid(s2, Output)) {
            return false;
        }

        // 2. Store all pins in an array for duplicate checking
        const uint8_t allPins[] = {readPin, ledOn, s0, s1, s2};

        // 3. Nested loop to compare each pin against all other pins
        for (int i = 0; i < 5; ++i) {
            for (int j = i + 1; j < 5; ++j) {
                if (allPins[i] == allPins[j]) {
                    return false; // If there is a duplicate
                }
            }
        }

        // 4. Check if the control pins are in ascending sequence
        if (s1 != s0 + 1 || s2 != s1 + 1) return false;

        return true; // All pins are unique and valid
    }

    static_assert(isValidMapping(), "Warning: Invalid or duplicate Pins for Sensors!");
    // TODO Edit this line <<IC_NAME>>.
    static_assert(numSensors <= 8, "Warning: MUX <<IC_NAME>> supports maximum 8 channels");
    static_assert(numSensors > 0, "Warning: numSensors must be at least 1");

    // ------------ Object Definition ------------
    TCRT5000() {
        pinMode(s0, OUTPUT);
        pinMode(s1, OUTPUT);
        pinMode(s2, OUTPUT);
        pinMode(ledOn, OUTPUT);
        pinMode(readPin, INPUT);
    }

    unsigned int averageRead() {
        int readValue = 0;
        for (int i = 0; i < 5; i++) readValue += analogRead(readPin);
        return readValue / 5;
    }

    unsigned int medianRead() {
        // Reading three times
        const int a = analogRead(readPin);
        const int b = analogRead(readPin);
        const int c = analogRead(readPin);

        return max(min(a, b), min(max(a, b), c)); // median
    }

    // TODO Add Exponential Moving Average (EMA) or Low-Pass Filter.

    void setMux(const uint8_t ch) {
        constexpr uint8_t numControlPins = 3; // MUX عدد أطراف التحكم في
        constexpr uint8_t maxChannel = (1 << numControlPins) - 1 ; // 2^3 - 1 = 7
        constexpr uint32_t baseMask = (1 << numControlPins) - 1; // Give 111 in binary
        constexpr uint32_t dynamicMask = baseMask << s0; // حساب ثابت به مواقع ألأطراف بقيمة 1 و الباقي 0

        if (ch <= maxChannel) {
            uint32_t reg = GPIO.out;
            reg &= ~dynamicMask;
            reg |= ((ch << s0) & dynamicMask);
            GPIO.out = reg;
        }
    }

    array<bool, numSensors> readSensors() {
        array<bool, numSensors> sensorsValue;

        setMux(0);
        for (int i = 0; i < numSensors; i++) {
            digitalWrite(ledOn, LOW);
            const unsigned int offRead = medianRead();
            digitalWrite(ledOn, HIGH);
            const unsigned int onRead = medianRead();
            digitalWrite(ledOn, LOW);

            sensorsValue[i] = (onRead - offRead) > threshold;
            if (i != numSensors - 1) setMux(i + 1);
        }
        return sensorsValue;
    }

    float getError() {
        auto sensorsValue = readSensors();
        int activeSensors = 0;
        float error = 0;

        constexpr float midPoint = (numSensors - 1) / 2.0f;
        // حساب الخطأ
        for (int i = 0; i < numSensors; i++) {
            if (sensorsValue[i]) {
                const float weight = static_cast<float>(i) - midPoint; // حساب وزن الحساس
                error += weight; // إضافة الخطأ
                activeSensors++; // عدد الحساسات التي فوق الخط
            }
        }
        if (activeSensors == 0) return lastError; // إذا فقد الخط
        const float currentError = error / static_cast<float>(activeSensors);
        lastError = currentError;
        return currentError;
    }
};
