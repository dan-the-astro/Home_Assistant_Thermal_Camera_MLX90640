// Non-secret firmware settings. Credentials live in include/secrets.h.
#pragma once

// ---- Identity ---------------------------------------------------------------
// Name shown for the device in Home Assistant.
#define DEVICE_NAME "Thermal Camera"
// Leave empty to derive a unique ID from the chip MAC (e.g. "thermalcam_a1b2c3").
// Set it explicitly if you run several cameras and want fixed entity IDs.
#define DEVICE_ID ""

// ---- MQTT topics ------------------------------------------------------------
// All topics are published under <MQTT_BASE_TOPIC>/<device id>/...
#define MQTT_BASE_TOPIC "thermalcam"
// Discovery prefix configured in the Home Assistant MQTT integration (default "homeassistant").
#define HA_DISCOVERY_PREFIX "homeassistant"
// Minimum interval between min/max sensor state updates (ms). Keeps the HA recorder sane
// at high frame rates. The camera image is still published for every frame.
#define STATE_PUBLISH_INTERVAL_MS 1000
// Minimum interval between camera images over MQTT (ms). 0 = publish every frame.
// Raise this (e.g. 2000) if you only want the MQTT camera for snapshots and use the
// MJPEG stream for live viewing, to reduce broker traffic.
#define MQTT_IMAGE_INTERVAL_MS 0

// ---- I2C / MLX90640 -----------------------------------------------------------
// XIAO ESP32S3 default I2C pins are D4 = GPIO5 (SDA) and D5 = GPIO6 (SCL).
#define I2C_SDA_PIN SDA
#define I2C_SCL_PIN SCL
// 400 kHz is safe with typical breakout wiring and supports refresh rates up to 16 Hz.
// Use 800000 or 1000000 (short wires, good pull-ups) if you want 32 / 64 Hz to keep up.
#define I2C_CLOCK_HZ 400000
#define MLX_I2C_ADDR 0x33
// Sensor refresh rate used until it is changed from Home Assistant (0.5, 1, 2, 4, 8, 16, 32, 64).
// Note: the MLX90640 produces one full frame per two refresh periods, so 8 Hz = ~4 images/s.
#define DEFAULT_REFRESH_RATE_HZ 8

// ---- Image rendering ----------------------------------------------------------
// The 32x24 sensor frame is bilinearly upscaled by this factor (8 -> 256x192 pixels).
#define IMAGE_SCALE 8
// JPEG quality 1..100 (higher = larger images).
#define JPEG_QUALITY 85
// Flip the image if it appears mirrored / upside down for your mounting orientation.
#define FLIP_HORIZONTAL 0
#define FLIP_VERTICAL 0
// 1 = colour range follows the min/max of each frame, 0 = fixed FIXED_RANGE_MIN..MAX.
#define AUTO_RANGE 1
// With AUTO_RANGE, never squeeze the palette into less than this many degrees C
// (prevents a uniform scene from turning into amplified noise).
#define MIN_RANGE_SPAN_C 4.0f
#define FIXED_RANGE_MIN_C 15.0f
#define FIXED_RANGE_MAX_C 40.0f
// Pixels outside this window are treated as sensor glitches and ignored for min/max.
#define VALID_TEMP_MIN_C -40.0f
#define VALID_TEMP_MAX_C 300.0f
// Set to 1 if hot areas show up blue instead of yellow/white (swaps red/blue for the JPEG encoder).
#define IMAGE_SWAP_RB 0

// ---- Local HTTP server (live MJPEG stream + snapshot + status page) ----------------
#define ENABLE_HTTP_STREAM 1
#define HTTP_PORT 80
