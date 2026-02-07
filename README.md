# Robot Control Node – Usage & Test Guide

This document explains **what the Robot Control Node does**, **how to build and run it**, and **how to test and verify that everything works as expected**.

---

## 1. Overview

The **Robot Control Node** runs on the Pico using **micro-ROS** and is responsible for:

* Receiving motion commands from ROS (`/cmd/vel`)
* Controlling **left & right motors** using **PID speed control**
* Reading wheel speed from **encoders**
* Publishing wheel RPM feedback
* Publishing **IMU data** (acceleration + angular velocity)

This node **does NOT do navigation or localization**.
It only executes **low-level control**.

---

## 2. Architecture (Big Picture)

```
ROS 2 (PC)
   |
   |  /cmd/vel  (geometry_msgs/Twist)
   v
Robot Control Node (Pico)
   |
   |-- Encoder HAL  ---> actual wheel RPM
   |-- PID          ---> throttle [-1 .. 1]
   |-- Motor HAL    ---> PWM + direction
   |
   |-- IMU Service  ---> accel + gyro
   |
   |--> /wheel/left/rpm   (Float32)
   |--> /wheel/right/rpm  (Float32)
   |--> /imu/data         (sensor_msgs/Imu)
```

---

## 3. Topics Summary

### Subscribed Topics

| Topic      | Type                  | Description                             |
| ---------- | --------------------- | --------------------------------------- |
| `/cmd/vel` | `geometry_msgs/Twist` | Desired robot linear & angular velocity |

### Published Topics

| Topic              | Type               | Description              |
| ------------------ | ------------------ | ------------------------ |
| `/wheel/left/rpm`  | `std_msgs/Float32` | Measured left wheel RPM  |
| `/wheel/right/rpm` | `std_msgs/Float32` | Measured right wheel RPM |
| `/imu/data`        | `sensor_msgs/Imu`  | Acceleration + gyro data |

---

## 4. Control Logic (What Happens Internally)

### 4.1 cmd_vel → Wheel RPM

* `linear.x` → forward/backward speed (m/s)
* `angular.z` → rotation around Z (rad/s)

Differential drive equations:

```
left_speed  = linear - (angular * wheel_base / 2)
right_speed = linear + (angular * wheel_base / 2)
```

Convert linear speed → RPM:

```
RPM = (speed / (2πR)) * 60
```

---

### 4.2 PID Speed Control

For each wheel:

```
error = target_rpm - measured_rpm
PID → throttle (-1 .. 1)
throttle → motor PWM + direction
```

* PID input = **encoder RPM**
* PID output = **motor throttle**
* IMU is **NOT used** in PID

---

### 4.3 IMU Publishing

* Runs on a **separate timer**
* Reads accel + gyro from MPU9250
* Publishes raw IMU data to ROS

No feedback from IMU to motors at this stage.

---

## 5. Timers & Callbacks

| Callback                 | Trigger        | Purpose                 |
| ------------------------ | -------------- | ----------------------- |
| `cmd_vel_callback`       | New `/cmd/vel` | Update target wheel RPM |
| `control_timer_callback` | 100 Hz         | Encoder → PID → Motor   |
| `imu_timer_callback`     | 50 Hz          | Read & publish IMU      |

Rule of thumb:

* **Callbacks** → react to external events
* **Timers** → run control loops

---

## 6. Build & Flash

### 6.1 Build

```bash
mkdir build
cd build
cmake ..
make
```

### 6.2 Flash to Pico

```bash
picotool load robot_control.uf2
```

(or via BOOTSEL + drag & drop)

---

## 7. Run micro-ROS Agent (PC)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Expected output:

```
[INFO] Agent running
[INFO] Client connected
```

---

## 8. Runtime Test Procedure

### 8.1 Check Node

```bash
ros2 node list
```

Expected:

```
/robot_control_node
```

---

### 8.2 Check Topics

```bash
ros2 topic list
```

Expected:

```
/cmd/vel
/wheel/left/rpm
/wheel/right/rpm
/imu/data
```

---

### 8.3 Send Motion Command

```bash
ros2 topic pub /cmd/vel geometry_msgs/Twist \
"{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Expected behavior:

* Robot moves forward
* RPM topics update

---

### 8.4 Rotate in Place

```bash
ros2 topic pub /cmd/vel geometry_msgs/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Expected behavior:

* Left & right wheels rotate opposite directions

---

### 8.5 Monitor Feedback

```bash
ros2 topic echo /wheel/left/rpm
ros2 topic echo /wheel/right/rpm
ros2 topic echo /imu/data
```

---

## 9. Expected Console Output (Pico)

```
Robot node ready - waiting for /cmd_vel.
```

