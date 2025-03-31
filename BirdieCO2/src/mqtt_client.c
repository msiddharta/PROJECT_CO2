#include "mqtt_client.h"
#include "esp_log.h"

static const char *TAG = "MQTT";
esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to MQTT broker");
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

void mqtt_start(void) {
    esp_mqtt_client_config_t config = {
        .uri = CONFIG_BROKER_URL  // Set in menuconfig
    };

    mqtt_client = esp_mqtt_client_init(&config);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void mqtt_publish_co2(uint16_t ppm) {
    if (!mqtt_client) return;

    char payload[16];
    snprintf(payload, sizeof(payload), "%u", ppm);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/co2", payload, 0, 1, 0);
}

void mqtt_publish_battery(float voltage, float soc) {
    if (!mqtt_client) return;

    char payload[64];
    snprintf(payload, sizeof(payload), "{\"voltage\":%.2f,\"soc\":%.1f}", voltage, soc);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/battery", payload, 0, 1, 0);
}
