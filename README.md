# ESP32 CO₂ Sensor & Servo Control Project

## Overview

This repository contains an ESP32-based firmware project developed using the **ESP-IDF** framework.  
The firmware:

- Reads **CO₂ concentration** using an **Infineon PAS CO₂ sensor** (I²C)
- Monitors **battery voltage and SoC** using a **MAX17048** fuel gauge
- Controls a **servo motor** based on sensor readings
- Publishes sensor data to an **MQTT** broker
- Allows user-triggered **CO₂ calibration** via a push-button

---

## Features

- **CO₂ Sensing**  
  Uses the Infineon PAS CO₂ sensor over I²C to measure CO₂ in parts per million (ppm).

- **Battery Monitoring**  
  Reads battery voltage and state-of-charge using MAX17048.

- **Servo Motor Control**  
  Controls a servo (0°/90°/180°) using ESP-IDF's MCPWM driver, depending on sensor thresholds.

- **MQTT Connectivity**  
  Connects to Wi-Fi and publishes CO₂, voltage, and SoC data to an MQTT broker.  
  Includes Home Assistant MQTT discovery support.

- **Push-button Calibration**  
  Triggers a forced calibration routine for the CO₂ sensor using a GPIO push-button.

- **Event-Driven Architecture**  
  Built using FreeRTOS tasks, semaphores, and event groups for clean modular logic.

- **Deep Sleep Support**  
  (Optional) ESP32 deep sleep mode configurable for ultra-low power operation.

---

## 🛠 Hardware Setup

| Component         | Description / Notes |
|------------------|---------------------|
| **ESP32 Board**  | Any ESP32 board that supports ESP-IDF |
| **CO₂ Sensor**   | Infineon PAS CO₂ via I²C (GPIO 3 = SDA, GPIO 4 = SCL) addressed on `0x28`|
| **Fuel Gauge**   | MAX17048 on I²C address `0x36` |
| **Servo Motor**  | Driven via MCPWM on GPIO 8 |
| **Push-button**  | For calibration trigger, on `GPIO_NUM_0` |
| **MQTT Broker**  | Configured in `mqtt.c` – supports MQTT URI, user/pass, IP |

---

## 📁 File Structure Overview

| File/Module           | Purpose |
|------------------------|---------|
| `main.c`              | Main task creation and event orchestration |
| `motor.[c/h]`         | MCPWM servo control |
| `co2sensor.[c/h]`     | Interface with PAS CO₂ sensor |
| `battery.[c/h]`       | Battery voltage and SoC readouts |
| `max17048_adv.[c/h]`  | Extended MAX17048 control (alerts, thresholds) |
| `i2c_bus.[c/h]`       | I²C bus driver and utilities |
| `mqtt.[c/h]`          | Wi-Fi, MQTT client, and Home Assistant integration |
| `sensor_state.h`      | Shared RTC memory state (e.g., last CO₂, servo angle) |
| `app_config.h`        | Configurable constants, GPIOs, intervals, thresholds |

---

## 🚀 Usage

### Normal Operation

On boot, the firmware:
- Initializes I²C
- Initializes the battery sensor
- Configures the CO₂ sensor
- Connects to Wi-Fi
- Starts MQTT
- Begins periodic CO₂ and battery monitoring

> Based on values in `app_config.h` like `CO2_HIGH_THRESHOLD` or `BATTERY_ALERT_MIN_VOLTAGE`, the servo is positioned accordingly and data is sent via MQTT.

---

### 📡 MQTT Topics

- `home/birdie/co2` → CO₂ in ppm  
- `home/birdie/voltage` → Battery voltage in V  
- `home/birdie/soc` → Battery state of charge in %

> Home Assistant discovery topics are published automatically on startup.

---

### 🔧 Calibration

Press the calibration button (GPIO 0) to:
- Stop CO₂ measurement task
- Run pressure setup, ABOC disable, calibration, and forced compensation
- Resume CO₂ measurements (restarting it)

---

### 🌀 Servo Behavior

| Condition         | Servo Angle | Config Macro           |
|------------------|-------------|-------------------------|
| CO₂ Low (normal) | `0°`        | `MOTOR_ANGLE_LOW`       |
| CO₂ High         | `90°`       | `MOTOR_ANGLE_MED`       |
| Battery Low      | `180°`      | `MOTOR_ANGLE_HIGH`      |

These can be modified in `app_config.h`.
