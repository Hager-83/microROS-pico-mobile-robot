#ifndef _PID_HPP_
#define _PID_HPP_

/******************************************* Include Part ******************************************** */
/**
 * @file PID.hpp
 * @brief PID controller class for motor speed control.
 * 
 * This file defines the MotorPID class which implements a PID controller
 * for controlling motor speed in RPM. Supports configurable Kp, Ki, Kd,
 * and allows updating the throttle based on measured motor speed.
 */
#include <stdint.h>

#define PI_value 3.14159
#define MAX_RPM 210.0

/**
 * @class MotorPID
 * @brief Implements a PID controller for motor speed control.
 *
 * MotorPID calculates proportional, integral, and derivative terms
 * to generate a throttle value for controlling motor speed.
 */
class MotorPID
{
    public:

     /**
     * @struct PIDINPUT
     * @brief Structure for initializing MotorPID controller.
     */

    /*
        Kp	0.3 → 1.2
        Ki	0.01 → 0.1
        Kd	0.0 → 0.05
    */
    typedef struct 
    {
        float kp;                    /**< Proportional gain */
        float kd;                    /**< Integral component */
        float ki;                    /**< Derivative component */
        float dt;                    /**< Sample time (in seconds) */
        float expected_speed;        /**< Desired speed in RPM */
    }PIDINPUT;

    /**
     * @struct PIDOutput
     * @brief Structure holding PID calculation results.
     */
    typedef struct 
    {
        float p;             /**< Proportional component */
        float i;             /**< Integral component */
        float d;             /**< Derivative component */
        float total;         /**< Sum of P + I + D */
    }PIDOUTPUT;

        /**
         * @brief Construct a new MotorPID object
         * 
         * @param PIDIn Pointer to PIDINPUT structure containing gains, dt, and target speed
         */
        MotorPID(PIDINPUT * PIDIn);

         /**
         * @brief Destroy the MotorPID object
         */
        virtual ~MotorPID();

        /**
         * @brief Set a new target speed for the motor.
         * 
         * @param rpm Desired motor speed in RPM
         * @param cw True for clockwise, false for counter-clockwise
         */
        void SetSpeedRPM(float rpm, bool cw);
        
        /**
         * @brief Update throttle based on measured motor speed
         * 
         * Calls ComputePID internally and applies the PID output to update throttle.
         * 
         * @param motor_speed Current motor speed in RPM
         * @return float New throttle value in range [-1.0, 1.0]
         */
        float UpdateThrottle(float motor_speed);

        /**
         * @brief Compute PID output based on current motor speed
         * 
         * Calculates P, I, and D components and returns a PIDOutput struct.
         * 
         * @param motor_speed Current motor speed in RPM
         * @return PIDOutput PID calculation result
         */
        PIDOUTPUT ComputePID(float motor_speed);

    private:

        /**
         * @brief Clamp a value between min and max
         * 
         * @param value Value to clamp
         * @param min Minimum allowed value
         * @param max Maximum allowed value
         * @return float Clamped value
         */
        float clamp(float value, float min, float max);

        float target_RPM_;     /**< Target motor speed in RPM */
        bool clock_wise_;      /**< Motor rotation direction */
        float throttle_;       /**< Current throttle value [-1.0, 1.0] */
        float dt_;             /**< Sample time (seconds) */
        float kp_;             /**< Proportional gain */
        float ki_;             /**< Integral gain */
        float kd_;             /**< Derivative gain */
        float error_;          /**< Current error (target - actual) */
        float last_error_;     /**< Previous error (for derivative calculation) */
        float error_sum_;      /**< Integral sum */
        float integral_max_;   /**< Maximum allowed integral term */
        float integral_min_;   /**< Minimum allowed integral term */

};

#endif  /*_PID_HPP_*/