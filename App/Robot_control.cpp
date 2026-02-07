#include <stdio.h>
#include <math.h>

/* micro-ROS */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

/* ROS msgs */
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>

/* Pico */
#include "pico/stdlib.h"
#include "../pico_uart_transports.h"

/* HAL + Services */
#include "encoder_hal.hpp"
#include "encoder_service.hpp"
#include "motor_hal.hpp"
#include "motor_service.hpp"
#include "PID.hpp"
#include "MPU9250_Service.hpp"

/* Robot parameters */
#define WHEEL_RADIUS_CM   3.5f
#define WHEEL_BASE_CM     25.0f
#define CONTROL_DT        0.01f   // 100 Hz

/* micro-ROS objects */
rcl_subscription_t cmd_vel_sub;

rcl_publisher_t left_rpm_pub;
rcl_publisher_t right_rpm_pub;
rcl_publisher_t imu_pub;

geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Float32 left_rpm_msg;
std_msgs__msg__Float32 right_rpm_msg;
sensor_msgs__msg__Imu imu_msg;

/* Hardware */
EncoderHAL left_encoder(ENCODER1_PIN_A, ENCODER1_PIN_B);
EncoderHAL right_encoder(ENCODER2_PIN_A, ENCODER2_PIN_B);

EncoderService left_encoder_service(left_encoder);
EncoderService right_encoder_service(right_encoder);

MotorHal left_motor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN);
MotorHal right_motor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN);

MotorService left_motor_service(left_motor);
MotorService right_motor_service(right_motor);

/* PID */
MotorPID::PIDINPUT left_pid_cfg  = {1.2f, 0.1f, 0.01f, CONTROL_DT, 0.0f};
MotorPID::PIDINPUT right_pid_cfg = {1.2f, 0.1f, 0.01f, CONTROL_DT, 0.0f};

MotorPID left_pid(&left_pid_cfg);
MotorPID right_pid(&right_pid_cfg);

/* IMU */
MPU9250_HAL imu_hal(i2c_default, MPU6500_DEFAULT_ADDRESS);
IMUService imu_service(imu_hal);

/* cmd_vel callback */
void cmd_vel_callback(const void * msg)
{
    const geometry_msgs__msg__Twist * twist =
        (const geometry_msgs__msg__Twist *)msg;

    float linear  = twist->linear.x * 100.0f;   // m/s → cm/s
    float angular = twist->angular.z;           // rad/s

    float left_speed  = linear - (angular * WHEEL_BASE_CM / 2.0f);
    float right_speed = linear + (angular * WHEEL_BASE_CM / 2.0f);

    float left_rpm  = (left_speed  / (2.0f * M_PI * WHEEL_RADIUS_CM)) * 60.0f;
    float right_rpm = (right_speed / (2.0f * M_PI * WHEEL_RADIUS_CM)) * 60.0f;

    left_pid.SetSpeedRPM(fabs(left_rpm),  left_rpm  >= 0);
    right_pid.SetSpeedRPM(fabs(right_rpm), right_rpm >= 0);

    printf("CMD_VEL target RPM updated. \n");
}

/* Control loop (100 Hz) */
void control_timer_callback(rcl_timer_t * timer, int64_t)
{
    if (!timer) return;

    float left_rpm  = left_encoder_service.encoder_getRPM();
    float right_rpm = right_encoder_service.encoder_getRPM();

    float left_throttle  = left_pid.UpdateThrottle(left_rpm);
    float right_throttle = right_pid.UpdateThrottle(right_rpm);

    left_motor_service.setTargetSpeed(left_throttle);
    right_motor_service.setTargetSpeed(right_throttle);

    left_rpm_msg.data  = left_rpm;
    right_rpm_msg.data = right_rpm;

    rcl_ret_t ret1 = rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL);
    rcl_ret_t ret2 = rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);
    if ((ret1 || ret2) != RCL_RET_OK) 
    {
        printf("Failed to publish Wheel RPM message\n");
    }
}

/* IMU timer (50 Hz) */
void imu_timer_callback(rcl_timer_t *, int64_t)
{
    IMUData data = imu_service.getAll();

    imu_msg.linear_acceleration.x = data.accel.x_g;
    imu_msg.linear_acceleration.y = data.accel.y_g;
    imu_msg.linear_acceleration.z = data.accel.z_g;

    imu_msg.angular_velocity.x = data.gyro.x_dps;
    imu_msg.angular_velocity.y = data.gyro.y_dps;
    imu_msg.angular_velocity.z = data.gyro.z_dps;

    rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (ret != RCL_RET_OK) 
    {
        printf("Failed to publish IMU message\n");
    }
    rcl_publish(&imu_pub, &imu_msg, NULL);
}

int main()
{
    stdio_init_all();
    sleep_ms(1500);

    printf("System started\n");

    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    /* Init hardware */
    left_encoder.encoder_init();
    right_encoder.encoder_init();
    left_encoder_service.encoder_start();
    right_encoder_service.encoder_start();

    while (!imu_hal.begin(PICO_DEFAULT_I2C_SDA_PIN,PICO_DEFAULT_I2C_SCL_PIN, 400000)) 
    {
        printf("IMU HAL begin failed, retrying...\n");
        sleep_ms(500);
    }
    while (!imu_service.begin()) 
    {
        printf("IMU Service begin failed, retrying...\n");
        sleep_ms(500);
    }

    /* micro-ROS */
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_timer_t control_timer, imu_timer;

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
    rclc_node_init_default(&node, "robot_control_node", "", &support);

    /* Topics */
    rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd/vel"
    );

    rclc_publisher_init_default(
        &left_rpm_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/wheel/left/rpm"
    );

    rclc_publisher_init_default(
        &right_rpm_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/wheel/right/rpm"
    );

    rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );

    rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(10), control_timer_callback
    );

    rclc_timer_init_default(
        &imu_timer, &support,
        RCL_MS_TO_NS(20), imu_timer_callback
    );
    // -------------------- Timer setup --------------------

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription( &executor, &cmd_vel_sub, &cmd_vel_msg,&cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &control_timer);
    rclc_executor_add_timer(&executor, &imu_timer);

    printf("Robot node ready - waiting for /cmd_vel. \n");

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    }
    // Cleanup (rarely reached)
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
