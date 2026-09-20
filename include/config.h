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
// at high frame rates.
#define STATE_PUBLISH_INTERVAL_MS 1000
// Minimum interval between camera images over MQTT (ms). 0 = publish every frame.
//
// Do not set this to 0 on a busy network. Each image is roughly 8 kB while the
// ESP32 TCP send buffer is 5760 bytes, so publishing every frame (about 4 per
// second at the default sensor rate) keeps the socket permanently saturated.
// The Home Assistant frontend only refreshes a still-image camera every ~10 s
// anyway, so anything below ~1000 ms is wasted broker traffic. Use the MJPEG
// stream for a genuinely live view.
#define MQTT_IMAGE_INTERVAL_MS 2000
// Retain the last camera image on the broker so Home Assistant has a picture
// immediately after a restart. Set to 0 if you would rather not have the broker
// rewrite its retained store on every image.
#define MQTT_IMAGE_RETAIN 1
// Size of the PubSubClient packet buffer. Must comfortably exceed the largest
// discovery payload; images bypass it through the streaming publish API.
#define MQTT_BUFFER_BYTES 2048
// Images are handed to the TCP stack in chunks of this size so a backed-up
// socket is noticed immediately instead of after a ten second stall.
#define MQTT_WRITE_CHUNK_BYTES 1024
// MQTT keepalive and CONNACK timeout, in seconds.
#define MQTT_KEEPALIVE_S 30
#define MQTT_SOCKET_TIMEOUT_S 5
// If the TCP send buffer stays full for this long the link is wedged. Drop the
// session and reconnect rather than waiting out the keepalive.
#define MQTT_SOCKET_STALL_MS 10000
// Reconnect backoff bounds (ms). The delay doubles after each failed attempt.
#define MQTT_RETRY_MIN_MS 2000
#define MQTT_RETRY_MAX_MS 60000

// ---- I2C / MLX90640 -----------------------------------------------------------
// XIAO ESP32S3 default I2C pins are D4 = GPIO5 (SDA) and D5 = GPIO6 (SCL).
#define I2C_SDA_PIN SDA
#define I2C_SCL_PIN SCL
// 400 kHz is safe with typical breakout wiring and supports refresh rates up to 16 Hz.
// Use 800000 or 1000000 (short wires, good pull-ups) if you want 32 / 64 Hz to keep up.
#define I2C_CLOCK_HZ 800000
#define MLX_I2C_ADDR 0x33
// Sensor refresh rate used until it is changed from Home Assistant (0.5, 1, 2, 4, 8, 16, 32, 64).
// Note: the MLX90640 produces one full frame per two refresh periods, so 8 Hz = ~4 images/s.
#define DEFAULT_REFRESH_RATE_HZ 8

// ---- Image rendering ----------------------------------------------------------
// The 32x24 sensor frame is bilinearly upscaled by this factor (8 -> 256x192 pixels).
#define IMAGE_SCALE 4
// JPEG quality 1..100 (higher = larger images).
#define JPEG_QUALITY 85
// Flip the image if it appears mirrored / upside down for your mounting orientation.
#define FLIP_HORIZONTAL 0
#define FLIP_VERTICAL 0
// 1 = colour range follows each frame's own temperature spread, 0 = fixed
// FIXED_RANGE_MIN..MAX.
#define AUTO_RANGE 1
// With AUTO_RANGE the palette ends come from these percentiles of the frame
// rather than from its single coldest and hottest pixel, so one noisy pixel at
// either end no longer sets the colour of everything else. Anything past them
// saturates to the end of the palette, which is usually what you want for a
// small hot object; the min/max sensors still report the true extremes.
#define RANGE_LOW_PERCENTILE 1.0f
#define RANGE_HIGH_PERCENTILE 99.0f
// How fast the palette range follows the scene (0..1 per frame, 1 = instantly).
// Undamped, anything that walks through the frame recolours the whole
// background. 0.25 settles in roughly ten frames.
#define RANGE_ADAPT_RATE 0.25f
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

// ---- Fixed-pattern noise ------------------------------------------------------
// The MLX90640 reads its array as two interleaved subpages through row- and
// column-banked electronics, and what the factory calibration leaves behind is a
// small offset that repeats every two pixels: banding between odd and even rows,
// a checkerboard between the two subpages, and weaker banding between columns.
// It is only a few tenths of a degree, but AUTO_RANGE spreads the palette over
// whatever the frame spans, so on a flat indoor scene those tenths land many
// palette entries apart and read as a grid over the picture.
// 1 = measure that pattern every frame and subtract it.
#define FPN_FILTER 1
// How fast the measured offsets follow the sensor (0..1 per frame). The pattern
// drifts with die temperature so it has to be tracked, but re-measuring it from
// scratch every frame would chase noise. 0.25 settles in about ten frames.
#define FPN_ADAPT_RATE 0.25f
// Ceiling on any single offset, in degrees C. The residual is tenths of a
// degree; a larger estimate means the scene fooled the estimator.
#define FPN_MAX_OFFSET_C 3.0f
// How far one row may stand out from the rest of the array before it is treated
// as scene rather than banding, as a multiple of the typical row. The sensor
// bands every row by a similar amount; a sharp edge only shows up on the rows
// it crosses. Lower is more cautious about erasing real detail.
#define FPN_PROFILE_OUTLIER 3.0f
// 1 = smooth the row-amplitude profile along the array before applying it.
// Measured both ways: turning it off tracks an odd row slightly better, but
// roughly triples how much a real edge is softened, so it stays on.
#define FPN_ROW_SMOOTHING 1
// The column offsets repeat every this many columns, because the array's column
// electronics are grouped that way. Measured on a real part the cycle is four
// columns wide and by far the largest of the three patterns, so a model that
// only alternates between neighbours cannot remove it. The solver in
// frame_filter.cpp is written for a period of four.
#define FPN_COLUMN_PERIOD 4
// 1 = replace pixels sitting more than DEAD_PIXEL_DELTA_C from the median of
// their neighbours. A few MLX90640 pixels read tens of degrees off; the factory
// marks them in EEPROM but the Adafruit library never applies that correction.
// Keep the threshold wide: at 32x24 a genuinely hot object can occupy a single
// pixel and a tighter filter would erase it.
#define REPAIR_DEAD_PIXELS 1
#define DEAD_PIXEL_DELTA_C 20.0f

// ---- WiFi -------------------------------------------------------------------
// How long to let the ESP32 core's own auto-reconnect work before forcing a
// reconnect ourselves (ms). Forcing one too eagerly aborts an association
// attempt that is already in progress and can keep the radio retrying forever.
#define WIFI_RECONNECT_GRACE_MS 20000

// ---- Local HTTP server (live MJPEG stream + snapshot + status page) ----------------
#define ENABLE_HTTP_STREAM 1
#define HTTP_PORT 80
