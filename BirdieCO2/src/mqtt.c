#include <stdio.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_netif_ip_addr.h"

#include "mqtt.h"

static const char *TAG = "MQTT";
esp_mqtt_client_handle_t mqtt_client = NULL;

#define WIFI_SSID "FRITZ!Box 5530 JF 2,4GHz"
#define WIFI_PASS "68077886860568218716"
#define WIFI_IP "mqtt://espuser:geheim123@192.168.178.44"
#define MAX_RETRY 10

static int retry_num = 0;
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to the AP");
        } else {
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization complete.");

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
}

void mqtt_discovery_publish() {
    if (!mqtt_client) return;

    // CO2 Sensor
    esp_mqtt_client_publish(mqtt_client,
        "homeassistant/sensor/birdie_co2/config",
        "{\"name\": \"Birdie CO2\", \"state_topic\": \"home/birdie/co2\", "
        "\"unit_of_measurement\": \"ppm\", \"device_class\": \"carbon_dioxide\", "
        "\"unique_id\": \"birdie_co2\", \"value_template\": \"{{ value | int }}\", "
        "\"device\": {\"identifiers\": [\"birdie_device\"], \"name\": \"Birdie Sensor\", \"model\": \"ESP32 CO2\", \"manufacturer\": \"GizmoLabs\"}}",
        0, 1, 1);

    // Voltage Sensor
    esp_mqtt_client_publish(mqtt_client,
        "homeassistant/sensor/birdie_voltage/config",
        "{\"name\": \"Birdie Voltage\", \"state_topic\": \"home/birdie/voltage\", "
        "\"unit_of_measurement\": \"V\", \"device_class\": \"voltage\", "
        "\"unique_id\": \"birdie_voltage\", \"value_template\": \"{{ value | float }}\", "
        "\"device\": {\"identifiers\": [\"birdie_device\"], \"name\": \"Birdie Sensor\", \"model\": \"ESP32 CO2\", \"manufacturer\": \"GizmoLabs\"}}",
        0, 1, 1);

    // Battery SoC Sensor
    esp_mqtt_client_publish(mqtt_client,
        "homeassistant/sensor/birdie_soc/config",
        "{\"name\": \"Birdie Battery SoC\", \"state_topic\": \"home/birdie/soc\", "
        "\"unit_of_measurement\": \"%\", \"device_class\": \"battery\", "
        "\"unique_id\": \"birdie_soc\", \"value_template\": \"{{ value | float }}\", "
        "\"device\": {\"identifiers\": [\"birdie_device\"], \"name\": \"Birdie Sensor\", \"model\": \"ESP32 CO2\", \"manufacturer\": \"GizmoLabs\"}}",
        0, 1, 1);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to MQTT broker");
            mqtt_discovery_publish();  // ← Publish config on connect
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
        .broker.address.uri = WIFI_IP
    };

    mqtt_client = esp_mqtt_client_init(&config);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void mqtt_stop(void) {
    if (mqtt_client) {
        esp_mqtt_client_stop(mqtt_client);
    }
}

void publish_data(uint16_t co2_ppm, float voltage, float soc) {
    if (!mqtt_client) return;

    char payload[64];

    snprintf(payload, sizeof(payload), "%u", co2_ppm);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/co2", payload, 0, 1, 1);

    snprintf(payload, sizeof(payload), "%.2f", voltage);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/voltage", payload, 0, 1, 1);

    snprintf(payload, sizeof(payload), "%.1f", soc);
    esp_mqtt_client_publish(mqtt_client, "home/birdie/soc", payload, 0, 1, 1);
}
