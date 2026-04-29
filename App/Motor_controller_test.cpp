#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

#include "pico/stdlib.h"
#include "../pico_uart_transports.h"

// HAL + Services
#include "encoder_hal.hpp"
#include "encoder_service.hpp"

#include "motor_hal.hpp"
#include "motor_service.hpp"

#include "PID.hpp"

// Robot parameters 
#define WHEEL_RADIUS_CM   3.5f
#define WHEEL_BASE_CM     25.0f
#define MAX_RPM           150.0f
#define CONTROL_DT        0.01f   // 100 Hz

// micro-ROS objects 
rcl_subscription_t cmd_vel_sub;

rcl_publisher_t left_rpm_pub;
rcl_publisher_t right_rpm_pub;
rcl_publisher_t left_pwm_pub;
rcl_publisher_t right_pwm_pub;

geometry_msgs__msg__Twist cmd_vel_msg;

std_msgs__msg__Float32 left_rpm_msg;
std_msgs__msg__Float32 right_rpm_msg;
std_msgs__msg__Float32 left_pwm_msg;
std_msgs__msg__Float32 right_pwm_msg;



// Hardware 
EncoderHAL left_encoder(ENCODER1_PIN_A, ENCODER1_PIN_B);
EncoderHAL right_encoder(ENCODER2_PIN_A, ENCODER2_PIN_B);


EncoderService left_encoder_service(left_encoder);
EncoderService right_encoder_service(right_encoder);

MotorHal left_motor(MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_EN);
MotorHal right_motor(MOTOR_RL_IN1, MOTOR_RL_IN2, MOTOR_RL_EN);

MotorService left_motor_service(left_motor);
MotorService right_motor_service(right_motor);

// PID objects 
MotorPID::PIDINPUT left_pid_cfg = {
    1.2f,
    0.1f,
    0.01f,
    CONTROL_DT,
    0.0f
};

MotorPID::PIDINPUT right_pid_cfg = {
    1.2f,
    0.1f,
    0.01f,
    CONTROL_DT,
    0.0f
};

MotorPID left_pid(&left_pid_cfg);
MotorPID right_pid(&right_pid_cfg);

//  Target RPM (set by cmd_vel) 
float target_left_rpm  = 0.0f;
float target_right_rpm = 0.0f;

// cmd_vel callback: converts Twist to target RPM
void cmd_vel_callback(const void * msg)
{
    const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msg;

    float linear  = twist->linear.x * 100.0f;    // m/s --> cm/s
    float angular = twist->angular.z;            // rad/s
 
    float left_speed_cm_s  = linear - (angular * WHEEL_BASE_CM / 2.0f);
    float right_speed_cm_s = linear + (angular * WHEEL_BASE_CM / 2.0f);

    target_left_rpm  = (left_speed_cm_s  / (2.0f * M_PI * WHEEL_RADIUS_CM)) * 60.0f;
    target_right_rpm = (right_speed_cm_s / (2.0f * M_PI * WHEEL_RADIUS_CM)) * 60.0f;

    // Update PID targets
    left_pid.SetSpeedRPM(fabs(target_left_rpm), target_left_rpm >= 0);
    right_pid.SetSpeedRPM(fabs(target_right_rpm), target_right_rpm >= 0);
}

// Control loop (100 Hz): read encoder --> PID --> apply throttle --> publish feedback
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (!timer) return;

    float left_rpm  = left_encoder_service.encoder_getRPM();
    float right_rpm = right_encoder_service.encoder_getRPM();

    float left_throttle  = left_pid.UpdateThrottle(left_rpm);
    float right_throttle = right_pid.UpdateThrottle(right_rpm);

    left_motor_service.setTargetSpeed(left_throttle);
    right_motor_service.setTargetSpeed(right_throttle);

    left_rpm_msg.data  = left_rpm;
    right_rpm_msg.data = right_rpm;
    left_pwm_msg.data  = left_throttle;
    right_pwm_msg.data = right_throttle;

    rcl_publish(&left_rpm_pub,  &left_rpm_msg,  NULL);
    rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);
    rcl_publish(&left_pwm_pub,  &left_pwm_msg,  NULL);
    rcl_publish(&right_pwm_pub, &right_pwm_msg, NULL);
}

// main
int main()
{
    stdio_init_all();
    sleep_ms(1500);
    printf("Motor Controller Node Started. \n");

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // -------------------- nitialize encoders --------------------
    left_encoder.encoder_init();
    right_encoder.encoder_init();
    left_encoder_service.encoder_start();
    right_encoder_service.encoder_start();

    // -------------------- micro-ROS variables --------------------
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_timer_t control_timer;

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        printf("Cannot reach micro-ROS agent --> exiting\n");
        return ret;
    }

    // -------------------- micro-ROS support & node --------------------
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motor_controller_node", "", &support);

    // -------------------- Subscriber: /cmd_vel --------------------
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    // -------------------- Publisher setup --------------------
    // -------------------- RPM feedback + applied throttle (PWM) 
    rclc_publisher_init_default(
        &left_rpm_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/left_wheel_rpm"
    );

    rclc_publisher_init_default(
        &right_rpm_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/right_wheel_rpm"
    );

    rclc_publisher_init_default(
        &left_pwm_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/left_motor_pwm"
    );

    rclc_publisher_init_default(
        &right_pwm_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/right_motor_pwm"
    );

    // -------------------- Timer setup --------------------
    rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(10),
        control_timer_callback
    );

    // -------------------- Executor setup --------------------
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,&cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &control_timer);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }

    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
