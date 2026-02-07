#ifndef ENCODER_CONFIG_HPP
#define ENCODER_CONFIG_HPP

/***************************************************************
 * Encoder GPIO Pin Assignments
 * Description:
 *     - Define GPIO pins for each quadrature encoder.
 *     - Supports multiple encoders (Encoder 1, Encoder 2).
 ****************************************************************/

// 1st encoder pins
#define ENCODER1_PIN_A 0
#define ENCODER1_PIN_B 1

// 2nd encoder pins
#define ENCODER2_PIN_A 12
#define ENCODER2_PIN_B 13

/***************************************************************
 * Encoder Parameters
 ***************************************************************/

// Encoder counts per revolution (CPR)
#define ENCODER_CPR 330

// Radius of the wheel in centimeters
#define WHEEL_RADIUS_CM 3.0f

#endif // ENCODER_CONFIG_HPP
