#ifndef MOTOR_HAL_HPP
#define MOTOR_HAL_HPP

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motor_config.hpp"

// Motor direction
enum class MotorState {
    STOP,
    FORWARD,   // CW
    BACKWARD   // CCW
};

class MotorHal {
public:
    MotorHal(uint in1, uint in2, uint en);

    void motorInit();
    void setMotorOutput(MotorState state, float duty); // duty: 0.0 -> 1.0
    void stop();

private:
    uint _in1, _in2, _en;
    uint _pwm_slice;
    bool _initialized = false;
};

#endif