#pragma once
#include <Arduino.h>
#include <array>
using namespace std;

template<
    uint8_t readPin, uint8_t ledOn, uint8_t s0, uint8_t s1,
    uint8_t s2, uint8_t s3, uint32_t threshold, uint8_t numSensors = 8
>
class QTR {
    float lastError = 0.0f;

public:
    // ------------ Compile-Time Safety ------------
    static constexpr bool isPinValid(const uint8_t pin) {
        // List of allowed pins in ESP32
        constexpr std::array<uint8_t, 17> valid_pins = {
            25, 26, 27, 14, 12, 13, 23, 22, 21, 19, 18, 5, 4, 2, 15, 17, 16
        };

        // Check if the pin belongs to the list
        for (const uint8_t p: valid_pins) { // NOLINT(*-use-anyofallof)
            if (p == pin) return true;
        }

        return false;
    }

    static constexpr bool isValidMapping() {
        // 1. Verify that all pins belong to the allowed ESP32 pin list
        if (!isPinValid(readPin) || !isPinValid(ledOn) || !isPinValid(s0) ||
            !isPinValid(s1) || !isPinValid(s2) || !isPinValid(s3)) {
            return false;
        }

        // 2. Store all pins in an array for duplicate checking
        const uint8_t allPins[] = {readPin, ledOn, s0, s1, s2, s3};

        // 3. Nested loop to compare each pin against all other pins
        for (int i = 0; i < 6; ++i) {
            for (int j = i + 1; j < 6; ++j) {
                if (allPins[i] == allPins[j]) {
                    return false; // If there is a duplicate
                }
            }
        }

        // 4. Check if the control pins are in ascending sequence
        if (s1 != s0 + 1 || s2 != s1 + 1 || s3 != s2 + 1) return false;

        return true; // All pins are unique and valid
    }

    static_assert(isValidMapping(), "Warning: Invalid or duplicate Pins for Sensors!");
    static_assert(numSensors <= 16, "Warning: MUX CD74HC4067 supports max 16 channels");
    static_assert(numSensors > 0, "Warning: numSensors must be at least 1");

    // ------------ Object Definition ------------
    QTR() {
        pinMode(s0, OUTPUT);
        pinMode(s1, OUTPUT);
        pinMode(s2, OUTPUT);
        pinMode(s3, OUTPUT);
        pinMode(ledOn, OUTPUT);
        pinMode(readPin, INPUT);
    }

    void setMux(const int i) {
        constexpr int numControlPins = 4; // MUX عدد أطراف التحكم في
        constexpr uint32_t baseMask = (1 << numControlPins) - 1; // يعطي 0b1111 (أي 0xF)
        constexpr uint32_t dynamicMask = baseMask << s0; // حساب ثابت به مواقع ألأطراف بقيمة 1 و الباقي 0

        // const uint32_t set_mask = (i << s0) & dynamicMask;
        // const uint32_t clear_mask = ((i ^ baseMask) << s0) & dynamicMask;
        // GPIO.out_w1ts = set_mask;   // HIGH تجعل ألأطراف المحددة قيمتهم
        // GPIO.out_w1tc = clear_mask; // LOW تجعل ألأطراف المحددة قيمتهم

        uint32_t reg = GPIO.out;
        reg &= ~dynamicMask;
        reg |= ((i << s0) & dynamicMask);
        GPIO.out = reg;
    }

    array<bool, numSensors> readSensors() {
        return {};
    }

    float errorLine() {
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


