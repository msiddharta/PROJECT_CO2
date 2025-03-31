#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdint.h>

void mqtt_app_start(void);
void mqtt_publish_co2(uint16_t ppm);
void mqtt_publish_battery(float voltage, float soc);

#endif // MQTT_CLIENT_H