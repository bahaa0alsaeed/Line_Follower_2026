#pragma once
#include <array>

// Bit flags for pin capabilities
enum PinType {
    ADC    = 1 << 0, // 0001
    PWM    = 1 << 1, // 0010
    Input  = 1 << 2, // 0100
    Output = 1 << 3, // 1000
};

// Compile-time lookup table for pin definitions
static constexpr std::array<uint8_t, 40> pinsTypes = [] {
    std::array<uint8_t, 40> arr{};

    // Pins that are Input-only and support Analog signals (e.g., ESP32 SENSOR_VP/VN)
    arr[36] = ADC | Input;
    arr[39] = ADC | Input;
    arr[34] = ADC | Input;
    arr[35] = ADC | Input;

    // Fully featured pins (ADC, PWM, I/O)
    arr[32] = ADC | PWM | Input | Output;
    arr[33] = ADC | PWM | Input | Output;
    arr[25] = ADC | PWM | Input | Output;
    arr[26] = ADC | PWM | Input | Output;
    arr[27] = ADC | PWM | Input | Output;
    arr[14] = ADC | PWM | Input | Output;
    arr[12] = ADC | PWM | Input | Output;
    arr[13] = ADC | PWM | Input | Output;

    // Digital pins supporting PWM and I/O (No ADC)
    arr[23] = PWM | Input | Output;
    arr[22] = PWM | Input | Output;
    arr[1]  = PWM | Input | Output;
    arr[3]  = PWM | Input | Output;
    arr[21] = PWM | Input | Output;
    arr[19] = PWM | Input | Output;
    arr[18] = PWM | Input | Output;
    arr[5]  = PWM | Input | Output;
    arr[17] = PWM | Input | Output;
    arr[16] = PWM | Input | Output;

    // Additional multifunction pins
    arr[4]  = ADC | PWM | Input | Output;
    arr[2]  = ADC | PWM | Input | Output;
    arr[15] = ADC | PWM | Input | Output;

    return arr;
}();

/**
 * @brief Validates if a specific pin supports the required features.
 * * @param pin The hardware pin number to check.
 * @param required The bitmask of required features (e.g., ADC | Output).
 * @return true if the pin exists and supports ALL required features.
 */
static constexpr bool isPinValid(const uint8_t pin, const uint8_t required) {
    // If no functionality is requested, consider the request invalid
    if (required == 0) return false;

    // Check if the pin index is within the array bounds
    if (pin < 40) {
        // Use Bitwise AND to check if all bits in 'required' are set in the pin's definition
        return ((pinsTypes[pin] & required) == required);
    }

    return false;
}
