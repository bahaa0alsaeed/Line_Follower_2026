#pragma once
#include "pins_validation.h"
#include <Arduino.h>
#include <array>
using namespace std;

struct SensorData {
        float error;          // PID Error
        int   activeCount;    // Number of active sensors
        bool  noneActive;     // Is all sensors off → Line loss
        bool  allActive;      // Most sensors are active → black area
        bool  leftEdgeOnly;   // Left sensor only → Sharp left turn
        bool  rightEdgeOnly;  // Right sensor only → Sharp right turn
};

// TODO Add a separate threshold for each sensor.
// TODO Add automatic calibration and normalization.

template<
    uint8_t ledOn, uint8_t s0, uint8_t s1, uint8_t s2,
    uint32_t threshold, uint8_t numSensors, const uint8_t* pinsArray
>
class TCRT5000 {
    float lastError = 0.0f;
    bool  inverted  = false;

public:
    // ------------ Compile-Time Safety ------------
    static constexpr bool isValidMapping() {
        // 1. Verify that all pins belong to the allowed ESP32 pin list
        if (!isPinValid(ledOn, Output | PWM) || !isPinValid(s0, Output) ||
            !isPinValid(s1, Output) || !isPinValid(s2, Output)) {
            return false;
        }
        for (int i = 0; i < numSensors; i++) { // Check Sensors
            if (!isPinValid(pinsArray[i], Input | ADC)) return false;
        }

        // 2. Store all pins in an array for duplicate checking
        constexpr size_t totalPinsCount = numSensors + 4;
        uint8_t allPins[totalPinsCount] = {ledOn, s0, s1, s2};
        for (size_t i = 0; i < numSensors; ++i) {
            allPins[4 + i] = pinsArray[i];
        }

        // 3. Nested loop to compare each pin against all other pins
        for (int i = 0; i < totalPinsCount; ++i) {
            for (int j = i + 1; j < totalPinsCount; ++j) {
                if (allPins[i] == allPins[j]) {
                    return false; // If there is a duplicate
                }
            }
        }

        // 4. Check if the control pins are in ascending sequence
        if (s1 != s0 + 1 || s2 != s1 + 1) return false;

        return true; // All pins are unique and valid
    }

    static_assert(isValidMapping(), "Warning: Invalid or Duplicate Pins for Sensors!");
    static_assert(numSensors <= 8, "Warning: MUX 74HC4067 supports maximum 8 channels for 3 selectors");
    static_assert(numSensors > 0, "Warning: numSensors must be at least 1");

    // ------------ Object Definition ------------
    TCRT5000() {
        pinMode(s0, OUTPUT);
        pinMode(s1, OUTPUT);
        pinMode(s2, OUTPUT);
        pinMode(ledOn, OUTPUT);
        for (int i = 0; i < numSensors; i++) {
            pinMode(pinsArray[i], Input);
        }
    }

    // void setInverted(const bool inv) { inverted = inv; }
    // bool isInverted() const    { return inverted; }

    uint16_t EMARead(const uint8_t numPin, const float alpha = 0.7) {
        static array<float, numSensors> sensorsLastRead = {-1.0};
        const float currentRead = analogRead(pinsArray[numPin]);

        if (sensorsLastRead[numPin] < 0) { // If first read
            sensorsLastRead[numPin] = currentRead;
            return static_cast<uint16_t>(currentRead);
        }

        const float smoothedRead = (currentRead * alpha) + (sensorsLastRead[numPin] * (1 - alpha));
        sensorsLastRead[numPin] = smoothedRead;
        return static_cast<uint16_t>(smoothedRead);
    }

    uint16_t calculateMedian(const uint16_t a, const uint16_t b, const uint16_t c) {
        return max(min(a, b), min(max(a, b), c));
    }

    void setMux(uint8_t ch) {
        ch ^= 0b111; // عكس القناة بسبب وجود بوابة NOT
        ch = ch - 0b1;
        constexpr uint8_t numControlPins = 3; // MUX عدد أطراف التحكم في
        constexpr uint8_t maxChannel = (1 << numControlPins) - 1 ; // 2^3 - 1 = 7
        constexpr uint32_t baseMask = (1 << numControlPins) - 1; // النتيجة: 111b
        constexpr uint32_t dynamicMask = baseMask << s0; // حساب ثابت به مواقع ألأطراف بقيمة 1 و الباقي 0

        if (ch <= maxChannel) {
            uint32_t reg = GPIO.out;
            reg &= ~dynamicMask;
            reg |= ((ch << s0) & dynamicMask);
            GPIO.out = reg;
        }
    }

    array<uint16_t, numSensors> readSensors() {

        array<uint16_t, numSensors> sensorsValue{};
        uint16_t on[3][numSensors];

        for (int pass = 0; pass < 3; pass++) {
            digitalWrite(ledOn, HIGH);
            delayMicroseconds(50);

            for (int i = 0; i < numSensors; i++) {
                setMux(i);
                on[pass][i] = analogRead(pinsArray[i]);
            }
        }

        for (int i = 0; i < numSensors; i++) {
            uint16_t finalOn = calculateMedian(on[0][i], on[1][i], on[2][i]);
            sensorsValue[i] = finalOn;
        }
        return sensorsValue;
    }

    SensorData getErrorDigital() {
        auto sensorsValue = readSensors();
        SensorData data{};

        constexpr float midPoint = (numSensors - 1) / 2.0f;
        // حساب الخطأ
        for (int i = 0; i < numSensors; i++) {
            if (sensorsValue[i] > threshold) {
                const float weight = static_cast<float>(i) - midPoint; // حساب وزن الحساس
                data.error += weight; // إضافة الخطأ
                data.activeCount++; // عدد الحساسات التي فوق الخط
            }
        }

        data.noneActive = (data.activeCount == 0);
        data.allActive = (static_cast<float>(data.activeCount) >= numSensors * 0.75f);

        // إذا فقد الخط
        if (data.noneActive) {
            data.error = lastError;
        }
        else {
            data.error /= static_cast<float>(data.activeCount);
            lastError = data.error;
        }

        const bool leftEdge = sensorsValue[0] > threshold || sensorsValue[1] > threshold;
        const bool rightEdge = sensorsValue[numSensors - 1] > threshold || sensorsValue[numSensors - 2] > threshold;

        data.leftEdgeOnly  = leftEdge  && data.error < -midPoint * 0.7f;
        data.rightEdgeOnly = rightEdge && data.error >  midPoint * 0.7f;

        Serial.println(data.error);

        return data;
    }

    SensorData getErrorAnalog() {

        auto sensorsValue = readSensors();

        SensorData data{};

        constexpr float midPoint = (numSensors - 1) / 2.0f;
        float weightedSum = 0.0f;
        float signalSum   = 0.0f;

        for (int i = 0; i < numSensors; i++) {
            const float signal = static_cast<float>(sensorsValue[i]);

            // للحالات الخاصة فقط
            if (signal > threshold) {
                data.activeCount++;
            }

            const float weight = static_cast<float>(i) - midPoint;
            weightedSum += weight * signal;
            signalSum += signal;
        }

        // الحالات الخاصة
        data.noneActive = (signalSum < 1.0f);
        data.allActive = (static_cast<float>(data.activeCount) >= numSensors * 0.75f);

        // حساب الخطأ
        if (data.noneActive) {
            data.error = lastError;
        }
        else {
            data.error = weightedSum / signalSum;
            lastError = data.error;
        }

        // Edge Detection
        const bool leftEdge = sensorsValue[0] > threshold || sensorsValue[1] > threshold;
        const bool rightEdge = sensorsValue[numSensors - 1] > threshold || sensorsValue[numSensors - 2] > threshold;

        data.leftEdgeOnly = leftEdge && data.error < -midPoint * 0.7f;
        data.rightEdgeOnly = rightEdge && data.error > midPoint * 0.7f;

        // Serial.println(data.error);

        return data;
    }

    void debugCompareMethods() {

        auto sensors = readSensors();

        SensorData binaryData = getErrorDigital();
        SensorData analogData = getErrorAnalog();

        Serial.println("====================================");

        Serial.print("Sensors: [ ");

        for (int i = 0; i < numSensors; i++) {

            Serial.print(sensors[i]);

            if (i < numSensors - 1)
                Serial.print(", ");
        }

        Serial.println(" ]");

        // Binary
        Serial.println("------ Binary Threshold ------");

        Serial.print("Error: ");
        Serial.println(binaryData.error, 4);

        // Analog
        Serial.println("------ Analog Weighted ------");

        Serial.print("Error: ");
        Serial.println(analogData.error, 4);

        Serial.println("====================================");
        Serial.println();

        delay(1000);
    }

    void debug() {
        const auto sensorsValue = readSensors();

        Serial.print("Sensors: [ ");

        for (int i = 0; i < numSensors; i++) {

            Serial.print(sensorsValue[i]);

            Serial.print(" (");

            Serial.print(sensorsValue[i] > threshold);

            Serial.print(")");

            if (i < numSensors - 1) {
                Serial.print(", ");
            }
        }

        Serial.println(" ]");
    }
};