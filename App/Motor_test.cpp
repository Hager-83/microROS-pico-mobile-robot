#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "../pico_uart_transports.h"   

#include <geometry_msgs/msg/twist.h>
//#include <std_msgs/msg/float32.h>

#include "motor_config.hpp"
#include "motor_hal.hpp"
#include "motor_service.hpp"  


// micro-ROS objects 
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// Motor objects (using pins from config)
MotorHal left_motor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN);
MotorHal right_motor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN);

MotorService left_service(left_motor);
MotorService right_service(right_motor);

// Kinematics parameters (based on robot) <--------
const float WHEEL_BASE_M  = 0.25f;    // Assume distance between wheels in meters (25 cm = 0.25 m)
const float MAX_SPEED_M_S = 1.0f;     // Maximum allowed speed in m/s

// cmd_vel callback: converts Twist to target speed for each motor
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear = msg->linear.x;   // Forward/backward speed in m/s
    float angular = msg->angular.z; // Rotation speed in rad/s

    // Differential drive kinematics
    float left_target  = linear - (angular * WHEEL_BASE_M / 2.0f);
    float right_target = linear + (angular * WHEEL_BASE_M / 2.0f);

    // Normalize to -1.0 → +1.0 (relative to max speed)
    left_target  /= MAX_SPEED_M_S;
    right_target /= MAX_SPEED_M_S;

    // Send target speed to each motor
    left_service.setTargetSpeed(left_target);
    right_service.setTargetSpeed(right_target);

    // Debug print
    printf("cmd_vel: lin=%.2f ang=%.2f → L=%.2f R=%.2f\n",
           linear, angular, left_target, right_target);
}


// Main 
int main() {
    // Initialize standard I/O
    stdio_init_all();
    sleep_ms(2000);
    printf("Pico micro-ROS Motor Node started\n");

    // Set custom serial transport for micro-ROS
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize motors 
    left_motor.motorInit();
    right_motor.motorInit();  

    // micro-ROS core setup
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;

    // Wait for micro-ROS agent (up to 120 seconds)
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        printf("Cannot reach micro-ROS agent → exiting\n");
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motors_node", "", &support);

    // Subscriber: /cmd_vel
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    // Executor: handles subscription and timer
    rclc_executor_init(&executor, &support.context, 2, &allocator); // 1 sub + 1 timer + margin
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    // rclc_executor_add_timer(&executor, &timer);

    printf("Motor node ready - waiting for /cmd_vel\n");

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup (rarely reached)
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}