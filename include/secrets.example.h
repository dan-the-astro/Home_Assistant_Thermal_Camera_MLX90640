// Copy this file to include/secrets.h and fill in your own values.
// include/secrets.h is ignored by git so your credentials stay private.
#pragma once

// ---- WiFi -----------------------------------------------------------------
#define WIFI_SSID     "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// ---- MQTT broker (the Mosquitto add-on in Home Assistant works well) -------
#define MQTT_HOST     "homeassistant.local"   // hostname or IP of the broker
#define MQTT_PORT     1883
#define MQTT_USER     "mqtt-user"             // leave "" for anonymous
#define MQTT_PASSWORD "mqtt-password"
