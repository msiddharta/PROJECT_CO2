#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Structure to encapsulate sensor-related state.
 *
 * This structure groups together all variables that are shared across tasks
 * for sensor operation, such as the last motor angle, last CO₂ measurement,
 * the debounce timer for the calibration button, and a flag to signal measurement stop.
 */
typedef struct {
    int last_motor_angle;         // Last known motor angle (in degrees); -1 indicates uninitialized.
    uint16_t last_co2_value;      // Last median CO₂ measurement (ppm).
    int64_t last_button_press_us; // Timestamp of the last button press (for debouncing).
    bool stop_measurement;        // Flag to signal the measurement task to stop.
} sensor_state_t;

#endif // SENSOR_STATE_H
