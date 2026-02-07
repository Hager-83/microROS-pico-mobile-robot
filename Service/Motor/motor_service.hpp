#ifndef MOTOR_SERVICE_HPP
#define MOTOR_SERVICE_HPP

#include "motor_hal.hpp"

class MotorService {
public:
    MotorService(MotorHal& driver);   
                                 
    void setTargetSpeed(float speed);  // -1.0 -> +1.0
    void stop();
    float getCurrentDuty() const;
    
private:
    MotorHal& _driver;
    float _current_duty = 0.0f;
};

#endif // MOTOR_SERVICE_HPP