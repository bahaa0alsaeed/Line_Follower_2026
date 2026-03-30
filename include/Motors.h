#pragma once
#include "pins_validation.h"
#include <Arduino.h>

// ------------------------- SingleMotor Class -------------------------
template<
    uint8_t rightPin, uint8_t leftPin, uint8_t speedPin,
    uint32_t maxSpeed, uint8_t ch, uint32_t frequency = 5000, uint8_t resolution = 8
>
class SingleMotor {
uint8_t currentSpeed = 0;

public:
    // ------------ Compile-Time Safety ------------
    static constexpr bool isValidMapping() {
        // 1. Verify the validity of all ports
        if (!isPinValid(rightPin, Output) || !isPinValid(leftPin, Output) ||
            !isPinValid(speedPin, Output | PWM)) return false;
        // 2. Verify that all pins are not duplicates
        if ((rightPin == leftPin) || (rightPin == speedPin) || (leftPin == speedPin)) return false;

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
    static_assert(ch <=15, "Warning: Invalid PWM channel (0-15 allowed)");
    static_assert(isFrequencyValid(frequency, resolution),
    "Warning: PWM Frequency is too high for the selected resolution!");

    // ------------ Object Definition ------------
    SingleMotor() {

        pinMode(rightPin, OUTPUT);
        pinMode(leftPin, OUTPUT);
        pinMode(speedPin, OUTPUT);

        ledcSetup(ch, frequency, resolution);
        ledcAttachPin(speedPin, ch);
    };

    void setMotorSpeed(const int s) {
        if (s >= 0) {
            currentSpeed = s;
            ledcWrite(ch, (s > maxSpeed) ? maxSpeed : s);
        }
    }

    void forward(const int speed = -1) {
        digitalWrite(rightPin,HIGH);
        digitalWrite(leftPin,LOW);
        if (speed >= 0) {
            setMotorSpeed(speed);
        }
    }

    void backward(const int speed = -1) {
        digitalWrite(rightPin,LOW);
        digitalWrite(leftPin,HIGH);
        if (speed >= 0) {
            setMotorSpeed(speed);
        }
    }

    void stop() {setMotorSpeed(0);}

    void softStart(const int speed) {
        forward(speed/4);
        delay(300);
        forward(speed/2);
        delay(300);
        forward(speed);
        delay(300);
    }

    void brake() {
        digitalWrite(rightPin, HIGH);
        digitalWrite(leftPin, HIGH);
        setMotorSpeed(maxSpeed);
    }
};

// ------------------------- DualMotors Class -------------------------
template<
    uint8_t R_IN1, uint8_t R_IN2, uint8_t R_PWM, uint8_t L_IN1,
    uint8_t L_IN2, uint8_t L_PWM , uint32_t maxSpeed, uint8_t R_CH = 0,
    uint8_t L_CH = 1, uint32_t frequency = 5000, uint8_t resolution = 8
>
class DualMotors {
public:
    // ------------ Compile-Time Safety ------------
    static constexpr bool isValidMapping() {
        // 1. Store all pins in an array for duplicate checking
        const uint8_t allPins[] = {
            R_IN1, R_IN2, R_PWM,
            L_IN1, L_IN2, L_PWM
        };

        // 2. Nested loop to compare each pin against all other pins
        for (int i = 0; i < 6; ++i) {
            for (int j = i + 1; j < 6; ++j) {
                if (allPins[i] == allPins[j]) {
                    return false; // If there is a duplicate
                }
            }
        }

        return true; // All pins are unique and valid
    }

    static_assert(isValidMapping(), "Warning: Duplicate Pins for Motors!");
    static_assert(L_CH != R_CH, "Warning: Duplicate PWM Channels for Motors!");

    // Define objects to enable their use in functions
    SingleMotor<R_IN1,R_IN2,R_PWM,maxSpeed,R_CH,frequency,resolution> right;
    SingleMotor<L_IN1,L_IN2,L_PWM,maxSpeed,L_CH,frequency,resolution> left;

    // ------------ Object Definition ------------
    explicit DualMotors() = default;

    void forward(int rightSpeed = -1, int leftSpeed = -1) {
        if (rightSpeed >= 0) {
            if (leftSpeed >= 0) {
                left.forward(leftSpeed);
                right.forward(rightSpeed);
            }else {
                left.forward(rightSpeed);
                right.forward(rightSpeed);
            }
        }else {
            left.forward(maxSpeed);
            right.forward(maxSpeed);
        }
    }

    void backward(int rightSpeed = -1, int leftSpeed = -1) {
        if (rightSpeed >= 0) {
            if (leftSpeed >= 0) {
                left.backward(leftSpeed);
                right.backward(rightSpeed);
            }else {
                left.backward(rightSpeed);
                right.backward(rightSpeed);
            }
        }else {
            left.backward(maxSpeed);
            right.backward(maxSpeed);
        }
    }

    void turnLeft(int rightSpeed = -1, int leftSpeed = -1) {
        if (rightSpeed >= 0) {
            if (leftSpeed >= 0) {
                left.backward(leftSpeed);
                right.forward(rightSpeed);
            }else {
                left.backward(rightSpeed);
                right.forward(rightSpeed);
            }
        }else {
            left.backward(maxSpeed);
            right.forward(maxSpeed);
        }
    }

    void turnRight(int rightSpeed = -1, int leftSpeed = -1) {
        if (rightSpeed >= 0) {
            if (leftSpeed >= 0) {
                left.forward(leftSpeed);
                right.backward(rightSpeed);
            }else {
                left.forward(rightSpeed);
                right.backward(rightSpeed);
            }
        }else {
            left.forward(maxSpeed);
            right.backward(maxSpeed);
        }
    }

    void stop() {
        left.stop();
        right.stop();
    }

    void softStart(const int speed) {
        right.forward(speed/4);
        left.forward(speed/4);
        delay(300);
        right.forward(speed/2);
        left.forward(speed/2);
        delay(300);
        right.forward(speed);
        left.forward(speed);
        delay(300);
    }

    void brake() {
        left.brake();
        right.brake();
    }
};
