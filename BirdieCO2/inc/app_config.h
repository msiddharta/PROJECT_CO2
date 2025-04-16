#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// ---------------------------------------------------------------------------
// Time Delays and Intervals (in milliseconds unless noted otherwise)
// ---------------------------------------------------------------------------
#define SENSOR_INIT_WAIT_TIME_MS      2000   // Wait time after sensor initialization
#define SENSOR_STABILIZATION_TIME_MS  200    // Sensor stabilization delay between operations
#define MEASUREMENT_INTERVAL_MS       5000   // Interval between CO₂ measurements
#define BATTERY_CHECK_INTERVAL_MS     (10 * 60 * 1000)  // Battery monitoring interval (10 minutes)

// Deep Sleep duration (in microseconds)
#define DEEP_SLEEP_TIME_US            (9 * 60 * 1000000ULL)  // Deep sleep duration (10 minutes)

// General time settings (in milliseconds)
#define TIME_SHORTEST_DELAY_MS        200    // Shortest delay (200ms)
#define TIME_SHORT_DELAY_MS           1000   // Short delay (1 second)
#define TIME_LONG_DELAY_MS            10000  // Long delay (10 seconds)
#define TIME_WAIT_FOR_SENSOR_MS       180000   // Wait time for sensor readiness (180 seconds)
#define TIME_WAIT_FOR_READYNESS_MS  5000   // Wait time for sensor readiness (5 seconds)

// ---------------------------------------------------------------------------
// Button and Debounce Settings
// ---------------------------------------------------------------------------
#define BUTTON_DEBOUNCE_TIME_US       300000  // Debounce time for button ISR (300ms)

// ---------------------------------------------------------------------------
// Sensor Thresholds and Calibration Values
// ---------------------------------------------------------------------------
#define CO2_LOW_THRESHOLD             350    // CO₂ low threshold (ppm)
#define CO2_HIGH_THRESHOLD            1000   // CO₂ high threshold (ppm)
#define SENSOR_PRESSURE_HPA           1017   // Pressure value (in hPa) used for sensor calibration
#define CO2_DIFF_THRESHOLD           500     // CO₂ difference threshold for recalculation (ppm)
// ---------------------------------------------------------------------------
// Battery (MAX17048) Specific Thresholds
// ---------------------------------------------------------------------------
#define BATTERY_ALERT_MIN_VOLTAGE     3.3f   // Minimum battery voltage (volts) for alerts
#define BATTERY_ALERT_MAX_VOLTAGE     4.2f   // Maximum battery voltage (volts) for alerts

// ---------------------------------------------------------------------------
// Semaphore and Mutex Settings
// ---------------------------------------------------------------------------
#define SENSOR_MUTEX_TIMEOUT_MS       10000  // Timeout for sensor mutex (10 seconds)

// ---------------------------------------------------------------------------
// Task Stack Sizes
// ---------------------------------------------------------------------------
#define BATTERY_TASK_STACK_SIZE       4096   // Stack size for battery monitoring task
#define CO2_CALIB_TASK_STACK_SIZE     4096   // Stack size for CO₂ calibration task
#define CO2_MEASURE_TASK_STACK_SIZE   4096   // Stack size for CO₂ measurement task
#define MQTT_TASK_STACK_SIZE          4096   // Stack size for MQTT task
#define MOTOR_TASK_STACK_SIZE         4096   // Stack size for motor control task

// ---------------------------------------------------------------------------
// Event Bit Definitions
// ---------------------------------------------------------------------------
#define EVENT_CO2_HIGH                (1 << 0)
#define EVENT_CO2_LOW                 (1 << 1)
#define EVENT_CO2_RECALIBRATE         (1 << 2)
#define EVENT_CO2_DATA_READY          (1 << 3)
#define EVENT_BATT_LOW                (1 << 4)
// ---------------------------------------------------------------------------
// GPIO Definitions
// ---------------------------------------------------------------------------
#define CALIB_BUTTON_GPIO             GPIO_NUM_0
#define LED_GPIO                      GPIO_NUM_13
#define GPIO_HIGH                   1
#define GPIO_LOW                    0
// ---------------------------------------------------------------------------
// Motor Angle Definitions
// ---------------------------------------------------------------------------
#define MOTOR_ANGLE_LOW               0
#define MOTOR_ANGLE_MED               90
#define MOTOR_ANGLE_HIGH              180
#define MOTOR_ANGLE_UNDEFINED           -1
// ---------------------------------------------------------------------------

#endif // APP_CONFIG_H
