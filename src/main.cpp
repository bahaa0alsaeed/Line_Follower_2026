#include <Arduino.h>
#include "TCRT5000.h"
#include "Motors.h"

// ---------------------------- CONSTANTS ------------------------------- //

// Motor pins
#define IN1 5
#define IN2 17
#define IN3 16
#define IN4 4
#define LEFT_SPEED_PIN 2
#define RIGHT_SPEED_PIN 18
constexpr int MAX_SPEED = 200;

// TCRT5000 pins
#define readingPin 36
#define ledOnPin 15
#define s0 25
#define s1 26
#define s2 27
constexpr int threshold = 2000;

// Switches
#define PB 33
#define DIP1 13
#define DIP2 12
#define DIP3 14

// ---------------------------- FUNCTIONS ------------------------------- //

DualMotors<
    IN1,IN2,RIGHT_SPEED_PIN,IN3,IN4,LEFT_SPEED_PIN,MAX_SPEED
> Robot;

TCRT5000 <
    readingPin,ledOnPin,s0,s1,s2,threshold
> Sensors;

// PID controller: takes error and returns correction
float PID(const float error, const float dt) {
    // ---------- PID Constants ----------
    static constexpr float KP = 2.0;
    static constexpr float KI = 0.0;
    static constexpr float KD = 0.0;
    static constexpr float INTEGRAL_LIMIT = 10.0;
    static float integral = 0.0;
    static float lastError = 0.0;
    static float previousDerivative = 0.0;

    // ---------- Main Logic ----------
    integral += error * dt;
    if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
    else if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

    static float alpha = 0.7;
    const float raw_derivative = (error - lastError) / dt;
    const float derivative = alpha * previousDerivative + (1 - alpha) * raw_derivative; // Low-Pass Filter

    previousDerivative = derivative;
    lastError = error;

    const float output = KP * error + KI * integral + KD * derivative;
    return constrain(output, -MAX_SPEED, MAX_SPEED);
}

void controlMotors(const float error) {
    if (error > 0) {
        const int rightSpeed = constrain(MAX_SPEED - error,-MAX_SPEED,MAX_SPEED);
        Robot.forward(rightSpeed,MAX_SPEED);
    }
    else if (error < 0) {
        const int leftSpeed = constrain(MAX_SPEED + error,-MAX_SPEED,MAX_SPEED);
        Robot.forward(MAX_SPEED,leftSpeed);
    }
    else {
        Robot.forward(MAX_SPEED);
    }
}
// ---------------------------- MAIN ------------------------------- //
void setup() {
    analogSetWidth(12);
    analogSetAttenuation(ADC_11db);

}

void loop() {

}