#include <stdio.h>                     // Standard I/O functions.
#include "esp_log.h"                   // ESP logging functions.
#include "mqtt_client.h"               // MQTT client APIs.
#include "esp_event.h"                 // ESP event loop functions.
#include "esp_wifi.h"                  // Wi-Fi functions.
#include "esp_netif.h"                 // Network interface functions.
#include "nvs_flash.h"                 // Non-volatile storage functions.
#include "esp_netif_ip_addr.h"         // IP address helper functions.

#include "mqtt.h"                      // Custom header for MQTT-related declarations.

// Logging tag for MQTT module messages.
static const char *TAG = "MQTT";

// Global handle for the MQTT client.
esp_mqtt_client_handle_t mqtt_client = NULL;

// Wi-Fi configuration constants.
#define WIFI_SSID "FRITZ!Box 5530 JF 2,4GHz"            // Wi-Fi network SSID.
#define WIFI_PASS "68077886860568218716"                  // Wi-Fi network password.
#define WIFI_IP "mqtt://espuser:geheim123@192.168.178.46"  // MQTT broker URI with credentials.
#define MAX_RETRY 10                                     // Maximum number of Wi-Fi reconnection attempts.

// Global variable to keep track of connection retry attempts.
static int retry_num = 0;

// Event group handle to manage Wi-Fi connection status.
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0                          // Bit flag indicating Wi-Fi connection is established.

/**
 * @brief Wi-Fi event handler.
 *
 * This function handles Wi-Fi-related events such as:
 * - Station start: Initiates connection to AP.
 * - Disconnection: Attempts reconnection until the maximum retry limit is reached.
 * - Got IP: Logs the assigned IP address and signals successful connection.
 *
 * @param arg         User data pointer (unused).
 * @param event_base  The event base identifier.
 * @param event_id    The event identifier.
 * @param event_data  Event-specific data.
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // When the Wi-Fi station starts, attempt to connect.
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // On disconnection, attempt to reconnect if under retry limit.
        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to the AP");
        } else {
            // Clear connected bit if retries exhausted.
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // When the device gets an IP address, log the IP and set the connected bit.
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize Wi-Fi in station mode.
 *
 * This function initializes NVS, network interfaces, the default event loop,
 * and configures Wi-Fi to operate in station mode. It also registers the Wi-Fi event
 * handler and waits for the Wi-Fi connection to be established.
 */
void wifi_init_sta(void) {
    // Initialize NVS (Non-Volatile Storage) flash.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create an event group to signal when Wi-Fi is connected.
    wifi_event_group = xEventGroupCreate();
    // Initialize the network interface.
    ESP_ERROR_CHECK(esp_netif_init());
    // Create the default event loop.
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Create default Wi-Fi station network interface.
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi with default configuration.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers for Wi-Fi events and IP events.
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Set Wi-Fi configuration with SSID and password.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    // Set Wi-Fi mode to Station.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // Configure the Wi-Fi connection.
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    // Start the Wi-Fi driver.
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization complete.");

    // Wait for the Wi-Fi connection event (max 10 seconds).
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
}

/**
 * @brief Publish MQTT discovery messages.
 *
 * This function publishes configuration messages for Home Assistant discovery.
 * It defines three sensors for CO2, Voltage, and Battery SoC.
 * The JSON payloads describe topics, units, and device information.
 */
void mqtt_discovery_publish() {
    if (!mqtt_client) return;

    // Publish configuration for CO2 sensor.
    esp_mqtt_client_publish(mqtt_client,
        "homeassistant/sensor/birdie_co2/config",
        "{\"name\": \"Birdie CO2\", \"state_topic\": \"home/birdie/co2\", "
        "\"unit_of_measurement\": \"ppm\", \"device_class\": \"carbon_dioxide\", "
        "\"unique_id\": \"birdie_co2\", \"value_template\": \"{{ value | int }}\", "
        "\"device\": {\"identifiers\": [\"birdie_device\"], \"name\": \"Birdie Sensor\", \"model\": \"ESP32 CO2\", \"manufacturer\": \"GizmoLabs\"}}",
        0, 1, 1);

    // Publish configuration for Voltage sensor.
    esp_mqtt_client_publish(mqtt_client,
        "homeassistant/sensor/birdie_voltage/config",
        "{\"name\": \"Birdie Voltage\", \"state_topic\": \"home/birdie/voltage\", "
        "\"unit_of_measurement\": \"V\", \"device_class\": \"voltage\", "
        "\"unique_id\": \"birdie_voltage\", \"value_template\": \"{{ value | float }}\", "
        "\"device\": {\"identifiers\": [\"birdie_device\"], \"name\": \"Birdie Sensor\", \"model\": \"ESP32 CO2\", \"manufacturer\": \"GizmoLabs\"}}",
        0, 1, 1);

    // Publish configuration for Battery SoC sensor.
    esp_mqtt_client_publish(mqtt_client,
        "homeassistant/sensor/birdie_soc/config",
        "{\"name\": \"Birdie Battery SoC\", \"state_topic\": \"home/birdie/soc\", "
        "\"unit_of_measurement\": \"%\", \"device_class\": \"battery\", "
        "\"unique_id\": \"birdie_soc\", \"value_template\": \"{{ value | float }}\", "
        "\"device\": {\"identifiers\": [\"birdie_device\"], \"name\": \"Birdie Sensor\", \"model\": \"ESP32 CO2\", \"manufacturer\": \"GizmoLabs\"}}",
        0, 1, 1);
}

/**
 * @brief MQTT event handler.
 *
 * Processes MQTT events such as connection, disconnection, and errors.
 * When connected, it triggers the publishing of discovery messages.
 *
 * @param handler_args   User data pointer (unused).
 * @param base           The event base.
 * @param event_id       The specific MQTT event identifier.
 * @param event_data     Additional event data.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to MQTT broker");
            // Publish discovery configuration messages when MQTT client connects.
            mqtt_discovery_publish();
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from MQTT broker");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            break;
        default:
            break;
    }
}

/**
 * @brief Start the MQTT client.
 *
 * Initializes the MQTT client with the broker configuration and registers the
 * MQTT event handler. Once initialized, the client is started.
 */
void mqtt_start(void) {
    // Set up MQTT client configuration.
    esp_mqtt_client_config_t config = {
        .broker.address.uri = WIFI_IP  // Broker URI, including credentials.
    };

    // Initialize the MQTT client with the configuration.
    mqtt_client = esp_mqtt_client_init(&config);
    // Register the event handler to process MQTT events.
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    // Start the MQTT client.
    esp_mqtt_client_start(mqtt_client);
}

/**
 * @brief Stop the MQTT client.
 *
 * Stops the MQTT client if it has been initialized.
 */
void mqtt_stop(void) {
    if (mqtt_client) {
        esp_mqtt_client_stop(mqtt_client);
    }
}

/**
 * @brief Publish sensor data over MQTT.
 *
 * Formats and publishes CO2 (in ppm), voltage (in V), and battery state of charge (SoC, in %)
 * to their respective topics.
 *
 * @param co2_ppm  CO2 concentration in parts per million.
 * @param voltage  Voltage value.
 * @param soc      Battery state of charge in percentage.
 */
void publish_data(uint16_t co2_ppm, float voltage, float soc) {
    if (!mqtt_client) return;

    char payload[64];

    // Publish CO2 sensor data.
    snprintf(payload, sizeof(payload), "%u", co2_ppm);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/co2", payload, 0, 1, 1);

    // Publish voltage sensor data.
    snprintf(payload, sizeof(payload), "%.2f", voltage);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/voltage", payload, 0, 1, 1);

    // Publish battery SoC sensor data.
    snprintf(payload, sizeof(payload), "%.1f", soc);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/soc", payload, 0, 1, 1);
}
