#pragma once
#include <array>

enum PinType {
    ADC    = 1 << 0, // 0001
    PWM    = 1 << 1, // 0010
    Input  = 1 << 2, // 0100
    Output = 1 << 3, // 1000
};

static constexpr std::array<uint8_t, 40> pinsTypes = [] {
    std::array<uint8_t, 40> arr{};

    arr[36] = ADC | Input;
    arr[39] = ADC | Input;
    arr[34] = ADC | Input;
    arr[35] = ADC | Input;

    arr[32] = ADC | PWM | Input | Output;
    arr[33] = ADC | PWM | Input | Output;
    arr[25] = ADC | PWM | Input | Output;
    arr[26] = ADC | PWM | Input | Output;
    arr[27] = ADC | PWM | Input | Output;
    arr[14] = ADC | PWM | Input | Output;
    arr[12] = ADC | PWM | Input | Output;
    arr[13] = ADC | PWM | Input | Output;

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

    arr[4]  = ADC | PWM | Input | Output;
    arr[2]  = ADC | PWM | Input | Output;
    arr[15] = ADC | PWM | Input | Output;

    return arr;
}();

static constexpr bool isPinValid(const uint8_t pin, const uint8_t required) {
    if (required == 0) return false;
    if (pin < 40) return ((pinsTypes[pin] & required) == required);

    return false;
}
