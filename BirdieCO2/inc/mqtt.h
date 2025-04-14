#ifndef MQTT_H
#define MQTT_H

#include <stdint.h>

/**
 * @file mqtt.h
 * @brief Header file for MQTT client and Wi-Fi station initialization.
 *
 * This file declares functions for:
 * - Initializing the Wi-Fi interface in station mode.
 * - Starting and stopping the MQTT client.
 * - Publishing sensor data to MQTT topics.
 * - Publishing MQTT discovery messages for Home Assistant integration.
 */

/**
 * @brief Initialize Wi-Fi in station mode.
 *
 * This function configures the ESP32 to operate as a Wi-Fi station.
 * It sets up the Wi-Fi driver, configures the connection parameters,
 * and waits until a connection to the access point is established.
 */
void wifi_init_sta(void);

/**
 * @brief Start the MQTT client.
 *
 * Initializes the MQTT client with the specified broker configuration,
 * registers the necessary event handlers, and starts the client to 
 * enable communication with the MQTT broker.
 */
void mqtt_start(void);

/**
 * @brief Stop the MQTT client.
 *
 * Stops the MQTT client if it is currently running. This terminates the 
 * MQTT connection with the broker.
 */
void mqtt_stop(void);

/**
 * @brief Publish sensor data over MQTT.
 *
 * Formats and publishes the sensor data for CO2 concentration, voltage,
 * and battery state of charge (SoC) to their corresponding MQTT topics.
 *
 * @param co2_ppm  CO2 concentration in parts per million.
 * @param voltage  Voltage measurement in volts.
 * @param soc      Battery state of charge in percentage.
 */
void publish_data(uint16_t co2_ppm, float voltage, float soc);

/**
 * @brief Publish MQTT discovery messages.
 *
 * Publishes MQTT discovery configuration messages that allow Home Assistant 
 * to automatically detect and configure the sensors. This function sends 
 * JSON payloads describing the sensors, their topics, units, and device information.
 */
void mqtt_discovery_publish(void);

#endif // MQTT_H
