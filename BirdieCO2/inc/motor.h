#ifndef MOTOR_H
#define MOTOR_H

#include "driver/mcpwm_prelude.h"

/**
 * @file motor.h
 * @brief Header file for servo motor control using MCPWM.
 *
 * This file defines configuration constants and function prototypes
 * for initializing and controlling a servo motor.
 */
#define SERVO_MIN_PULSEWIDTH_US 500     // Minimum pulse width in microseconds (min servo position)
#define SERVO_MAX_PULSEWIDTH_US 2500    // Maximum pulse width in microseconds (max servo position)  
#define SERVO_MIN_DEGREE        0       // Minimum servo angle in degrees
#define SERVO_MAX_DEGREE        180     // Maximum servo angle in degrees
#define SERVO_PULSE_GPIO        8       // GPIO pin used for the servo's PWM signal    
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000    // Timer resolution for MCPWM (1 µs precision)  
#define SERVO_TIMEBASE_PERIOD   20000       // PWM period in microseconds (typically 50Hz)

/**
 * @brief Initialize the servo motor control.
 *
 * This function configures the MCPWM module to generate the PWM signal
 * required for controlling the servo motor.
 */
void motor_init();

/**
 * @brief Set the servo motor's angle.
 *
 * This function adjusts the PWM duty cycle to position the servo motor at
 * the specified angle.
 *
 * @param angle The desired angle (in degrees) to set the servo motor.
 */
void motor_set_angle(int angle);

#endif // MOTOR_H
