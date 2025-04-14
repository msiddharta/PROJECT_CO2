#include "motor.h"                      // Include motor interface header.
#include "esp_log.h"                    // Include logging capabilities.
#include "driver/mcpwm_prelude.h"       // Include MCPWM driver for servo control.
#include "esp_mac.h"                    // (Optional) Include MAC functions if needed.

// Define a tag for logging messages from the motor module.
static const char *TAG = "MOTOR";

// Global handle for the MCPWM comparator that controls pulse width.
static mcpwm_cmpr_handle_t comparator;

/**
 * @brief Convert a given angle (in degrees) to a comparator value.
 *
 * This inline function converts an angle between SERVO_MIN_DEGREE and SERVO_MAX_DEGREE
 * to an appropriate pulse width value (in microseconds) corresponding to the servo position.
 *
 * The formula scales the angle linearly between SERVO_MIN_PULSEWIDTH_US and SERVO_MAX_PULSEWIDTH_US.
 *
 * @param angle The angle in degrees.
 * @return uint32_t The calculated comparator value corresponding to the desired pulse width.
 */
static inline uint32_t angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * 
           (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + 
           SERVO_MIN_PULSEWIDTH_US;
}

/**
 * @brief Initialize the servo motor control using MCPWM.
 *
 * This function initializes all the necessary MCPWM components:
 * - Creates and configures a timer with the specified resolution and period.
 * - Creates an operator and connects it to the timer.
 * - Creates a comparator for dynamically adjusting the PWM pulse width.
 * - Creates a generator associated with the PWM output pin.
 *
 * It sets the default pulse width corresponding to 0° and configures actions:
 * - When the timer counts up and empties, the generator output is set HIGH.
 * - When the comparator triggers on compare, the generator output is set LOW.
 * Finally, the timer is enabled and started.
 */
void motor_init() {
    ESP_LOGI(TAG, "Initializing motor...");

    // Declare and initialize MCPWM timer with configuration settings.
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,                                    // Use group 0.
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,           // Use default clock source.
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,      // Timer resolution (1us precision).
        .period_ticks = SERVO_TIMEBASE_PERIOD,            // PWM period in ticks (e.g., 20000us).
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,          // Count upwards.
    };
    // Create new timer with the above configuration.
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Create an MCPWM operator for managing MCPWM actions.
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,                                    // Associate operator with group 0.
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    // Connect the operator to the previously created timer.
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // Set up a comparator to manage the pulse width for the servo.
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,                  // Automatically update comparator value on timer event "TEZ".
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // Set up a generator to produce the PWM signal on a specific GPIO.
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,                 // GPIO pin where servo control signal is output.
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // Initialize the comparator with default pulse width for angle 0°.
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

    // Configure the generator to set the output high at the start of each PWM period.
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

    // Configure the generator to set the output low when the comparator event occurs.
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // Enable the timer.
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    // Start the timer in continuous mode.
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Motor initialized.");
}

/**
 * @brief Set the servo motor to a specific angle.
 *
 * This function calculates the appropriate comparator value for the desired angle,
 * logs the action, and applies the new compare value for the MCPWM comparator, which
 * adjusts the PWM pulse width and, in turn, the servo position.
 *
 * @param angle The target servo angle in degrees.
 */
void motor_set_angle(int angle) {
    ESP_LOGI(TAG, "Setting angle to: %d", angle);
    // Update the comparator with the new value corresponding to the given angle.
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));
}
