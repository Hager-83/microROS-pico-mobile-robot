#include "PID.hpp"

MotorPID::MotorPID(PIDINPUT * PIDIn):target_RPM_(PIDIn->expected_speed),
                                    clock_wise_(true),
                                    throttle_(0.0f),
                                    dt_(PIDIn->dt),
                                    kp_(PIDIn->kp),
                                    ki_(PIDIn->ki),
                                    kd_(PIDIn->kd),
                                    error_(0.0f),
                                    last_error_(0.0f),
                                    error_sum_(0.0f)
{
    if(ki_!= 0.0f)
    {
        integral_max_ =  MAX_RPM / ki_;
        integral_min_ = -MAX_RPM / ki_;
    }
    else
    {
        integral_max_ = 0.0f;
        integral_min_ = 0.0f;
    }
}
/***************************************************************************************************************************************************** */

void MotorPID::SetSpeedRPM(float rpm, bool cw)
{
    target_RPM_ = cw ? rpm : -rpm;
    clock_wise_ = cw; 
    throttle_ = clamp(target_RPM_/ MAX_RPM, -1.0f, 1.0f);
    error_sum_  = 0.0;
}
/***************************************************************************************************************************************************** */

MotorPID::PIDOUTPUT MotorPID::ComputePID(float motor_speed)
{
    PIDOUTPUT PID_out;

    error_ = target_RPM_- motor_speed;  // PorPortinal
    error_sum_ += error_ * dt_;         // Integral
    error_sum_ = clamp(error_sum_, integral_min_, integral_max_);

    PID_out.p = error_ * kp_;

    if(dt_ != 0.0f)
    {
        PID_out.d = kd_ * ( (error_ - last_error_) /dt_ );
    }
    else
    {
        PID_out.d = 0.0f;
    }
    
    PID_out.i = error_sum_ * ki_;

    PID_out.total = (PID_out.d + PID_out.i + PID_out.p);

    last_error_ = error_;

    return(PID_out);
}
/***************************************************************************************************************************************************** */

float MotorPID::UpdateThrottle(float motor_speed)
{
    
    float pid_Process_result = (this->ComputePID(motor_speed)).total;

    float delta = pid_Process_result / MAX_RPM ;
    float t =  throttle_+ delta;

    t = clamp(t,-1.0f,1.0f);
    throttle_ = t;

    return(t);
}
/***************************************************************************************************************************************************** */

float MotorPID::clamp(float value, float min, float max) 
{
    return (value > max) ? max : (value < min) ? min : value;
}
/***************************************************************************************************************************************************** */

MotorPID::~MotorPID() {}
/***************************************************************************************************************************************************** */