#include <stdio.h>
#include <math.h>  // if needed in future calculations

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>

#include "pico/stdlib.h"
#include "../pico_uart_transports.h"     
//#include "pico_usb_transport.h" 

#include "encoder_config.hpp"                  
#include "encoder_hal.hpp"
#include "encoder_service.hpp"
/*
#include "HAL/Encoder/encoder_config.hpp"
#include "HAL/Encoder/encoder_hal.hpp"
#include "service/Encoder/encoder_service.hpp"*/



// micro-ROS objects 
rcl_publisher_t left_speed_pub;
rcl_publisher_t right_speed_pub;
rcl_publisher_t left_rpm_pub;
rcl_publisher_t right_rpm_pub;

std_msgs__msg__Float32 left_speed_msg;
std_msgs__msg__Float32 right_speed_msg;
std_msgs__msg__Float32 left_rpm_msg;
std_msgs__msg__Float32 right_rpm_msg;


// Encoder objects
EncoderHAL left_encoder(ENCODER1_PIN_A, ENCODER1_PIN_B);
EncoderHAL right_encoder(ENCODER2_PIN_A, ENCODER2_PIN_B);

EncoderService left_service(left_encoder);
EncoderService right_service(right_encoder);


// ────────────────────────────────────────────────
// Timer callback - reads values and publishes them
// ────────────────────────────────────────────────
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;  // suppress unused parameter warning

    if (timer == NULL)
    {
        return;
    }
    else{}

    // Read current values from encoder services
    float left_speed      = left_service.encoder_getSpeedCmS();
    float right_speed     = right_service.encoder_getSpeedCmS();
    float left_rpm        = left_service.encoder_getRPM();
    float right_rpm       = right_service.encoder_getRPM();

    // Fill messages
    left_speed_msg.data  = left_speed;
    right_speed_msg.data = right_speed;
    left_rpm_msg.data    = left_rpm;
    right_rpm_msg.data   = right_rpm;

    // Publish all
    rcl_ret_t ret;

    ret = rcl_publish(&left_speed_pub, &left_speed_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish left speed. \n");
    }

    ret = rcl_publish(&right_speed_pub, &right_speed_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish right speed. \n");
    }

    ret = rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish left RPM. \n");
    }

    ret = rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish right RPM.\n");
    }

    // print every 1 second for debugging
    static uint32_t counter = 0;
    counter++;
    if (counter % 10 == 0)  
    {
        printf("[%.1fs] L: %.2f cm/s  %.1f RPM   ---  R: %.2f cm/s  %.1f RPM\n",
               (float)last_call_time / 1e9f,  // Convert nanosec to sec
               left_speed, left_rpm,
               right_speed, right_rpm);
    }
    else{}
}


// ────────────────────────────────────────────────
// Main function
// ────────────────────────────────────────────────
int main()
{
    // Initialize USB 
    stdio_init_all();
    sleep_ms(1500);           // give time for USB serial to connect
    printf("Pico micro-ROS Encoders node started\n");

    // Set custom serial transport for micro-ROS
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize encoders and start periodic calculation (100 ms timer inside service)
    left_encoder.encoder_init();
    right_encoder.encoder_init();

    left_service.encoder_start();
    right_service.encoder_start();

    // ────────────────────────────────────────────────
    // micro-ROS core initialization
    // ────────────────────────────────────────────────
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
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

    // Create support and node
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "encoders_node", "", &support);

    // Create publishers
    rclc_publisher_init_default(
        &left_speed_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_wheel_speed"
    );

    rclc_publisher_init_default(
        &right_speed_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_wheel_speed"
    );

    rclc_publisher_init_default(
        &left_rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_wheel_rpm"
    );

    rclc_publisher_init_default(
        &right_rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_wheel_rpm"
    );

    // Create timer (publish rate)
    const unsigned int timer_period_ms = 150;   // 150 ms = ~6.67 Hz
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_period_ms),
        timer_callback
    );

    // Create executor (handles timer callbacks)
    rclc_executor_init(&executor, &support.context, 5, &allocator);   // 1 timer + margin
    rclc_executor_add_timer(&executor, &timer);

    printf("micro-ROS ready - publishing encoder data\n");

    // Main loop - spin executor
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup (rarely reached)
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}