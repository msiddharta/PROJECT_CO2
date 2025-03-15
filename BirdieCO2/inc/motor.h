#ifndef MOTOR_H
#define MOTOR_H

#include "driver/mcpwm_prelude.h"

// Servo configuration constants
#define SERVO_MIN_PULSEWIDTH_US 500  
#define SERVO_MAX_PULSEWIDTH_US 2500  
#define SERVO_MIN_DEGREE        0   
#define SERVO_MAX_DEGREE        180    
#define SERVO_PULSE_GPIO        8        
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  
#define SERVO_TIMEBASE_PERIOD   20000    

// Function prototypes
void motor_init();
void motor_set_angle(int angle);

#endif // MOTOR_H