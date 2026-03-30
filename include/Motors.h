#pragma once
#include "pins_validation.h"
#include <Arduino.h>

// ------------------------- SingleMotor Class -------------------------
template<
    uint8_t rightPin, uint8_t leftPin, uint32_t maxSpeed, uint8_t ch1,
    uint8_t ch2, uint32_t frequency = 5000, uint8_t resolution = 8
>
class SingleMotor {
    int currentSpeed = 0;

public:
    // ------------ Compile-Time Safety ------------
    static constexpr bool isValidMapping() {
        // 1. Verify the validity of all ports
        if (!isPinValid(rightPin, Output) || !isPinValid(leftPin, Output)) return false;
        // 2. Verify that all pins are not duplicates
        if (rightPin == leftPin) return false;

        return true;
    }

    static constexpr bool isFrequencyValid(const uint32_t freq, const uint8_t res) {
        // Basic ESP32 limit check
        if (res < 1 || res > 20) return false;

        // Calculate max frequency for the given resolution
        // Using 80MHz as base clock
        const double max_f = 80000000.0 / (1 << res);

        return static_cast<double>(freq) <= max_f;
    }

    static constexpr uint32_t PWM_MAX = (1UL << resolution) - 1;// UL -> unsigned long

    static_assert(isValidMapping(), "Warning: Invalid or duplicate Pins for Motors!");
    static_assert(resolution >= 1 && resolution <= 20, "Warning: PWM Resolution must be 1-20 bits");
    static_assert(maxSpeed <= PWM_MAX, "Warning: maxSpeed exceeds resolution limit!");
    static_assert((ch2 <=15) && (ch1 <= 15), "Warning: Invalid PWM channel (0-15 allowed)");
    static_assert(isFrequencyValid(frequency, resolution),
    "Warning: PWM Frequency is too high for the selected resolution!");

    // ------------ Object Definition ------------
    SingleMotor() {

        pinMode(rightPin, OUTPUT);
        pinMode(leftPin, OUTPUT);

        ledcSetup(ch1, frequency, resolution);
        ledcAttachPin(rightPin, ch1);

        ledcSetup(ch2, frequency, resolution);
        ledcAttachPin(leftPin, ch2);
    };

    void set(int speed) {
        speed = constrain(speed, -maxSpeed, maxSpeed);

        if (speed >= 0) {
            ledcWrite(ch1, speed);
            ledcWrite(ch2, 0);
            currentSpeed = speed;
        } else {
            ledcWrite(ch1, 0);
            ledcWrite(ch2, -speed); // -speed to make it positive
            currentSpeed = speed;
        }
    }

    void stop() {
        ledcWrite(ch1, 0);
        ledcWrite(ch2, 0);
        currentSpeed = 0;
    }

    void rampTo(int targetSpeed, const int step = 5) {
        targetSpeed = constrain(targetSpeed, -maxSpeed, maxSpeed);

        if (targetSpeed > currentSpeed)
            currentSpeed += step;
        else if (targetSpeed < currentSpeed)
            currentSpeed -= step;

        set(currentSpeed);
    }

    void softStart(const int speed) {
        set(speed/4);
        delay(300);
        set(speed/2);
        delay(300);
        set(speed);
        delay(300);
    }

    void brake() {
        ledcWrite(ch1, maxSpeed);
        ledcWrite(ch2, maxSpeed);
        currentSpeed = maxSpeed;
    }
};

// ------------------------- DualMotors Class -------------------------
template<
    uint8_t R_IN1, uint8_t R_IN2, uint8_t L_IN1, uint8_t L_IN2,
    uint32_t maxSpeed, uint8_t R_CH1, uint8_t R_CH2, uint8_t L_CH1,
    uint8_t L_CH2, uint32_t frequency = 5000, uint8_t resolution = 8
>
class DualMotors {
public:
    // ------------ Compile-Time Safety ------------
    static constexpr bool isValidMapping() {
        // 1. Store all pins in an array for duplicate checking
        const uint8_t rightPins[] = {R_IN1, R_IN2,};
        const uint8_t leftPins[] = {L_IN1, L_IN2};

        // 2. Nested loop to compare each pin against all other pins
        for (int i = 0; i < 2; ++i) {
            for (int j = i + 1; j < 2; ++j) {
                if (rightPins[i] == leftPins[j]) {
                    return false; // If there is a duplicate
                }
            }
        }

        return true; // All pins are unique and valid
    }

    static constexpr bool checkChannels() {
        const uint8_t channels[4] = {L_CH1,L_CH2,R_CH1,R_CH2};

        // Nested loop to compare each channel against all other channels
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                if (channels[i] == channels[j]) {
                    return false; // If there is a duplicate
                }
            }
        }
        return true;
    }

    static_assert(isValidMapping(), "Warning: Duplicate Pins for Motors!");
    static_assert(checkChannels(), "Warning: Duplicate PWM Channel for Motors!");

    // ---------------- Motors ----------------
    SingleMotor<R_IN1,R_IN2,maxSpeed,R_CH1,R_CH2,frequency,resolution> right;
    SingleMotor<L_IN1,L_IN2,maxSpeed,L_CH1,L_CH2,frequency,resolution> left;

    // ------------ Object Definition ------------
    void set(int rightSpeed, int leftSpeed) {
        right.set(rightSpeed);
        left.set(leftSpeed);
    }

    void forward(const int speed, const int leftSpeed = -1) {
        if (leftSpeed < 0)
            set(speed, speed);
        else
            set(speed, leftSpeed);
    }

    void backward(const int speed, const int leftSpeed = -1) {
        if (leftSpeed < 0)
            set(-speed, -speed);
        else
            set(-speed, -leftSpeed);
    }

    void turnRight(const int speed, const int leftSpeed = -1) {
        if (leftSpeed < 0)
            set(-speed, speed);
        else
            set(-speed, leftSpeed);
    }

    void turnLeft(const int speed, const int leftSpeed = -1) {
        if (leftSpeed < 0)
            set(speed,-speed);
        else
            set(speed,-leftSpeed);
    }

    void stop() {
        right.stop();
        left.stop();
    }

    void brake() {
        right.brake();
        left.brake();
    }

    // Non-blocking soft start
    void rampTo(int rTarget, int lTarget, int step = 5) {
        rTarget = constrain(rTarget, -maxSpeed, maxSpeed);
        lTarget = constrain(lTarget, -maxSpeed, maxSpeed);

        // For Right Motor
        if (rTarget > right.currentSpeed)
            right.currentSpeed += step;
        else if (rTarget < right.currentSpeed)
            right.currentSpeed -= step;

        // For Left Motor
        if (lTarget > left.currentSpeed)
            left.currentSpeed += step;
        else if (lTarget < left.currentSpeed)
            left.currentSpeed -= step;

        set(right.currentSpeed,left.currentSpeed);
    }
};
