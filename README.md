# MLX90640 Thermal Camera for Home Assistant (XIAO ESP32S3)

Firmware for a Seeed Studio **XIAO ESP32S3** with a Melexis **MLX90640** 32x24 thermal camera on the
default I2C bus. The device connects over WiFi and appears in Home Assistant automatically through
MQTT discovery with these entities:

| Entity | Type | Description |
| --- | --- | --- |
| Thermal Image | `camera` | False-colour JPEG of the latest frame, published every 2 s (`MQTT_IMAGE_INTERVAL_MS`) |
| Maximum Temperature | `sensor` (°C) | Hottest pixel in the frame |
| Minimum Temperature | `sensor` (°C) | Coldest pixel in the frame |
| Frame Rate | `select` (config) | MLX90640 refresh rate: 0.5, 1, 2, 4, 8, 16, 32 or 64 Hz, stored in flash |

The device also runs a small web server with a real-time **MJPEG stream** (`http://<ip>/stream`),
a snapshot endpoint and a status page. See [Live view](#live-view-mjpeg-stream) for why you want it.

## Hardware

| MLX90640 module | XIAO ESP32S3 |
| --- | --- |
| VIN / VCC (3.3 V) | 3V3 |
| GND | GND |
| SDA | D4 (GPIO5) |
| SCL | D5 (GPIO6) |

Notes:

- Most MLX90640 breakouts (Adafruit, Pimoroni, GY-MCU90640, generic "MLX90640 module") already
  have I2C pull-up resistors. If yours does not, add 4.7 kΩ pull-ups from SDA and SCL to 3V3.
- Keep the I2C wires short. The default bus speed is 400 kHz; 32 Hz and 64 Hz sensor refresh rates
  need 800 kHz to 1 MHz (`I2C_CLOCK_HZ` in `include/config.h`) and clean wiring.
- The sensor address is 0x33 by default (`MLX_I2C_ADDR`).

## Building and flashing

The project uses [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

1. Copy `include/secrets.example.h` to `include/secrets.h` and enter your WiFi SSID/password and
   MQTT broker host, port, username and password. This file is git-ignored.
2. Optionally adjust `include/config.h` (device name, image scale, colour range, pins and so on).
3. Connect the XIAO over USB-C and build/upload:

   ```
   pio run -t upload
   pio device monitor
   ```

   In VS Code use the PlatformIO toolbar: **Build**, **Upload**, **Monitor**.

The serial monitor (115200 baud) shows the sensor serial number, the IP address and a frame
statistics line every 5 seconds.

If upload fails because the board is not detected, hold the **B** (boot) button while pressing
**R** (reset) to enter the bootloader, then upload again.

## Adding the device to Home Assistant

### 1. MQTT broker and integration

If you do not already have a broker:

1. In Home Assistant go to **Settings → Add-ons → Add-on store**, install **Mosquitto broker** and start it.
2. Create a Home Assistant user for the camera (**Settings → People → Users → Add user**), for
   example `mqtt-user`. The Mosquitto add-on lets any Home Assistant user log in to the broker.
3. Go to **Settings → Devices & services**. The **MQTT** integration is normally discovered
   automatically once Mosquitto runs; if not, click **Add integration → MQTT** and point it at the broker.

Put the broker hostname or IP and the user credentials into `include/secrets.h`. Leave the
discovery prefix in the MQTT integration at its default `homeassistant` (or change
`HA_DISCOVERY_PREFIX` in `config.h` to match).

### 2. Power on the camera

Within a few seconds of booting the firmware publishes its discovery messages. A device named
**Thermal Camera** (configurable with `DEVICE_NAME`) appears under
**Settings → Devices & services → MQTT**, with the camera, the two temperature sensors and the
Frame Rate select. No YAML is required.

The device page also has a **Visit device** link that opens the camera's own web page.

If the device does not appear:

- Check the serial monitor for `[mqtt] connected` and `[mqtt] discovery published`.
- Use **MQTT → Configure → Listen to a topic** with `thermalcam/#` to confirm messages arrive.
- Make sure the MQTT integration's discovery option is enabled.

### 3. Dashboard cards

Entity IDs are derived from the device name, for example `camera.thermal_camera_thermal_image`,
`sensor.thermal_camera_maximum_temperature`, `sensor.thermal_camera_minimum_temperature` and
`select.thermal_camera_frame_rate`.

```yaml
type: picture-glance
title: Thermal camera
camera_image: camera.thermal_camera_thermal_image
camera_view: auto
entities:
  - sensor.thermal_camera_maximum_temperature
  - sensor.thermal_camera_minimum_temperature
  - select.thermal_camera_frame_rate
```

## Live view (MJPEG stream)

The MQTT camera entity receives every frame, so snapshots, notifications and automations always
have the latest image. However, the Home Assistant frontend only refreshes still-image cameras
about every 10 seconds, so a dashboard card of the MQTT camera will not look live.

For a genuinely live view, add the stream served by the device:

1. **Settings → Devices & services → Add integration → MJPEG IP Camera**.
2. MJPEG URL: `http://<device-ip>/stream` (or `http://thermalcam-xxxxxx.local/stream`, the hostname
   is printed on the serial monitor).
3. Still image URL: `http://<device-ip>/snapshot.jpg`. Leave authentication empty.

This creates a second camera entity that plays the stream at the full sensor frame rate in
dashboards and the more-info dialog. Giving the XIAO a fixed IP (DHCP reservation) in your router is
recommended so the URL never changes.

Endpoints on the device:

| URL | Content |
| --- | --- |
| `/` | Status page with the live stream and the current min/max/fps |
| `/stream` | `multipart/x-mixed-replace` MJPEG stream (up to 4 concurrent clients) |
| `/snapshot.jpg` | Latest frame as JPEG |
| `/state.json` | `{"min":..,"max":..,"fps":..,"rate":"8 Hz","clients":n}` |

Set `ENABLE_HTTP_STREAM 0` in `config.h` to disable the web server entirely.

## Frame rate

The **Frame Rate** select changes the MLX90640 refresh rate on the fly and stores the choice in
flash, so it survives reboots. Two things to know:

- The MLX90640 refreshes its two sub-pages alternately, so a full image is produced every two
  refresh periods: **8 Hz gives about 4 images per second**, 16 Hz about 8, and so on. The `fps`
  field in `thermalcam/<id>/state` and on the status page shows the real image rate.
- Higher rates need a faster I2C bus. At the default 400 kHz the sensor keeps up to 16 Hz. For
  32/64 Hz set `I2C_CLOCK_HZ` to `800000` or `1000000`. Higher refresh rates also increase sensor
  noise; 4 to 8 Hz gives the cleanest image for stationary scenes.

Images go to MQTT every `MQTT_IMAGE_INTERVAL_MS` (2 s by default), not on every frame. The sensor
still runs at the rate you select, and the MJPEG stream still shows every frame; only the MQTT
camera entity is paced. This matters more than it looks: an image is around 8 kB while the ESP32's
TCP send buffer is 5760 bytes, so publishing four of them per second keeps the socket permanently
saturated. The Home Assistant frontend only refreshes a still-image camera every ten seconds or so,
so the extra traffic buys nothing. Set the interval to `0` to publish every frame only if you know
the link can take it.

The temperature sensors are updated at most once per `STATE_PUBLISH_INTERVAL_MS` (1 s) so the
recorder database stays small.

## MQTT topics

All topics live under `thermalcam/<device id>/` (`MQTT_BASE_TOPIC`). The device id defaults to
`thermalcam_` followed by the last three bytes of the chip MAC, or `DEVICE_ID` if set.

| Topic | Direction | Payload |
| --- | --- | --- |
| `.../image` | device → HA | JPEG bytes (retained) |
| `.../state` | device → HA | `{"min":21.3,"max":34.8,"fps":3.9,"refresh_rate":"8 Hz","frames":1234}` |
| `.../refresh_rate` | device → HA | current option, e.g. `8 Hz` (retained) |
| `.../refresh_rate/set` | HA → device | new option: `8 Hz`, or just `8` |
| `.../status` | device → HA | `online` / `offline` (last will, used for availability) |
| `homeassistant/<component>/<device id>/<object>/config` | device → HA | discovery configs (retained) |

The firmware listens on `homeassistant/status` and re-announces itself when Home Assistant restarts.

## Image rendering options (`include/config.h`)

| Setting | Default | Effect |
| --- | --- | --- |
| `IMAGE_SCALE` | 8 | Bilinear upscale factor: 8 → 256x192 px JPEG |
| `JPEG_QUALITY` | 85 | JPEG quality (size vs. detail) |
| `AUTO_RANGE` | 1 | Palette spans each frame's min..max (with `MIN_RANGE_SPAN_C` minimum) |
| `FIXED_RANGE_MIN_C` / `MAX_C` | 15 / 40 | Palette range when `AUTO_RANGE` is 0 |
| `FLIP_HORIZONTAL` / `FLIP_VERTICAL` | 0 / 0 | Mirror the image for your mounting orientation |
| `IMAGE_SWAP_RB` | 0 | Set to 1 if hot areas render blue instead of yellow/white |

The palette is an "ironbow" gradient (black → purple → red → orange → yellow → white).

## MQTT and connection options (`include/config.h`)

| Setting | Default | Effect |
| --- | --- | --- |
| `MQTT_IMAGE_INTERVAL_MS` | 2000 | Minimum gap between camera images. `0` publishes every frame |
| `MQTT_IMAGE_RETAIN` | 1 | Retain the last image so HA has a picture right after a restart |
| `MQTT_BUFFER_BYTES` | 2048 | PubSubClient packet buffer. Only discovery payloads use it |
| `MQTT_WRITE_CHUNK_BYTES` | 1024 | Chunk size for streaming an image to the socket |
| `MQTT_KEEPALIVE_S` / `MQTT_SOCKET_TIMEOUT_S` | 30 / 5 | Keepalive and CONNACK timeout |
| `MQTT_RETRY_MIN_MS` / `MQTT_RETRY_MAX_MS` | 2000 / 60000 | Reconnect backoff, doubling after each failure |
| `MQTT_SOCKET_STALL_MS` | 10000 | Drop and rebuild the session if the send buffer stays full this long |
| `WIFI_RECONNECT_GRACE_MS` | 20000 | How long to let the ESP32's own auto-reconnect work before forcing one |

## Troubleshooting

- **`MLX90640 not found`**: check 3V3/GND/SDA/SCL, the pull-ups and the address. The firmware
  retries every 5 seconds, so fixing the wiring does not require a reboot.
- **`getFrame failed`** with a non-zero code, or a garbled image at high rates: lower the frame
  rate or raise `I2C_CLOCK_HZ`; shorten the wires. After 10 consecutive errors the sensor is
  re-initialised automatically.
- **Image mirrored / upside down**: set `FLIP_HORIZONTAL` and/or `FLIP_VERTICAL`.
- **Wrong colours** (cold = yellow, hot = blue): set `IMAGE_SWAP_RB 1`.
- **Entities unavailable**: the broker holds the last will; the device is marked unavailable when
  its MQTT connection drops. Check WiFi signal and the serial monitor.
- **The camera drops off every few seconds** and the serial monitor shows `image truncated`,
  `dropping connection` or `Connection reset by peer`: the device is producing images faster than
  the link to the broker can absorb them. An MQTT packet declares its length up front, so a
  half-sent image leaves the broker consuming everything that follows as the rest of that payload
  until it gives up and closes the socket. Raise `MQTT_IMAGE_INTERVAL_MS`, lower `JPEG_QUALITY` or
  `IMAGE_SCALE`, and check the WiFi signal. The firmware recovers by rebuilding the session rather
  than continuing on a corrupted one, and counts how often it has had to.
- **Home Assistant feels slow or the entities keep reloading**: watch the `drops` and
  `images skipped` counters in the `[frame]` log line. Anything other than a slowly growing
  `images skipped` means the MQTT link is unhealthy. Discovery configs are retained and published
  once per boot, so a device that re-announces repeatedly is reconnecting repeatedly.
- **Several cameras**: set a distinct `DEVICE_ID` and `DEVICE_NAME` per board.

## Project layout

```
platformio.ini            board, framework and library dependencies
include/config.h          tunable settings (pins, image, MQTT topics, HTTP server)
include/secrets.h         WiFi and MQTT credentials (git-ignored; copy from secrets.example.h)
src/main.cpp              WiFi, MQTT discovery, sensor loop, frame rate handling
src/thermal_image.*       32x24 → upscaled false-colour bitmap renderer
src/stream_server.*       MJPEG / snapshot / status HTTP server (FreeRTOS task on core 0)
```

Libraries: Adafruit MLX90640, PubSubClient, ArduinoJson, plus the esp32-camera JPEG encoder that
ships with the Arduino ESP32 core.
