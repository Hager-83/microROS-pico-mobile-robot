#include <cmath>
#include "motor_service.hpp"

// Constructor
MotorService::MotorService(MotorHal& driver)
    : _driver(driver) {}


// Set motor speed with direction
void MotorService::setTargetSpeed(float speed) {
    
    if (speed > 1.0f)
    {
        speed = 1.0f;
    }
    else{}
    if (speed < -1.0f)
    {
        speed = -1.0f;
    } 
    else{}

    _current_duty = std::fabs(speed);

    if (speed > 0) 
    {
        _driver.setMotorOutput(MotorState::FORWARD, speed);
    } 
    else if (speed < 0) 
    {
        _driver.setMotorOutput(MotorState::BACKWARD, -speed);
    } 
    else 
    {
        stop();
    }
}

// Stop the motor
void MotorService::stop() {
    _current_duty = 0.0f;
    _driver.stop();
}

float MotorService::getCurrentDuty() const {
    return _current_duty;
}