#pragma once

class PID {
    float integral = 0.0;
    float lastError = 0.0;
    float previousDerivative = 0.0;
    const float KP; // Proportional gain
    const float KI; // Integral gain
    const float KD; // Derivative gain
    const float INTEGRAL_LIMIT; // Anti-windup threshold

public:
    PID(const float kp, const float ki, const float kd, const float I_limit = 10.0) :
    KP(kp), KI(ki), KD(kd), INTEGRAL_LIMIT(I_limit) {};

    // PID controller: takes error and returns correction.
    float computePID(const float error, const float dt) {
        // Accumulate and clamp integral to prevent windup
        integral += error * dt;
        if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
        else if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

        // Apply low-pass filter to derivative to reduce noise
        static float alpha = 0.7;
        const float raw_derivative = (error - lastError) / dt;
        const float derivative = alpha * previousDerivative + (1 - alpha) * raw_derivative;

        previousDerivative = derivative;
        lastError = error;

        // Combine P, I, and D components
        return KP * error + KI * integral + KD * derivative;
    }
};