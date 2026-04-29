#ifndef MOTOR_CONFIG_HPP
#define MOTOR_CONFIG_HPP

/* ================================================================
 * Motor Pin Configuration — 4 motors via 2x L298N
 *
 * Reserved pins (encoders + comms):
 *   GPIO 6,7   → Encoder 1 (FL)
 *   GPIO 9,10  → Encoder 2 (RL)
 *   GPIO 12,13 → Encoder 3 (FR)
 *   GPIO 15,16 → Encoder 4 (RR)
 *
 * L298N #1 — LEFT SIDE
 *   Channel A → Motor FL (front-left)
 *   Channel B → Motor RL (rear-left)
 *
 * L298N #2 — RIGHT SIDE
 *   Channel A → Motor FR (front-right)
 *   Channel B → Motor RR (rear-right)
 * ================================================================ */

/* ── L298N #1 Channel A → Motor FL ──────────────────────────── */
#define MOTOR_FL_IN1    6
#define MOTOR_FL_IN2    7
#define MOTOR_FL_EN     8     // PWM

/* ── L298N #1 Channel B → Motor RL ──────────────────────────── */
#define MOTOR_RL_IN1    9
#define MOTOR_RL_IN2    10
#define MOTOR_RL_EN     11    // PWM

/* ── L298N #2 Channel A → Motor FR ──────────────────────────── */
#define MOTOR_FR_IN1    12
#define MOTOR_FR_IN2    13
#define MOTOR_FR_EN     14    // PWM

/* ── L298N #2 Channel B → Motor RR ──────────────────────────── */
#define MOTOR_RR_IN1    15
#define MOTOR_RR_IN2    16
#define MOTOR_RR_EN     17    // PWM

/* ── PWM resolution ──────────────────────────────────────────── */
#define PWM_WRAP  1000

#endif // MOTOR_CONFIG_HPP