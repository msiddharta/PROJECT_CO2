#ifndef MQTT_H
#define MQTT_H

#include <stdint.h>

void wifi_init_sta(void);
void mqtt_start(void);
void mqtt_stop(void);
void publish_data(uint16_t co2_ppm, float voltage, float soc);
void mqtt_discovery_publish(void);

#endif // MQTT_H
