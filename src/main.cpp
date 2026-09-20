// XIAO ESP32S3 + MLX90640 thermal camera for Home Assistant.
//
// - Reads 32x24 temperature frames from the MLX90640 over I2C.
// - Renders a false-colour JPEG and publishes it as an MQTT camera.
// - Publishes min / max temperature sensors.
// - Exposes the sensor refresh rate as a "Frame Rate" select entity.
// - Everything is announced via MQTT discovery, so the device shows up automatically.
// - Additionally serves an MJPEG stream on http://<device>/stream for a true live view.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_MLX90640.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <lwip/sockets.h>
#include <math.h>

#include "img_converters.h"  // fmt2jpg() from the esp32-camera component bundled with the core

#include "config.h"
#include "frame_filter.h"
#include "secrets.h"
#include "stream_server.h"
#include "thermal_image.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

// ---------------------------------------------------------------------------
// Refresh-rate options offered to Home Assistant
// ---------------------------------------------------------------------------
struct RateOption {
  const char *label;
  mlx90640_refreshrate_t rate;
  float hz;
};

static const RateOption RATES[] = {
    {"0.5 Hz", MLX90640_0_5_HZ, 0.5f}, {"1 Hz", MLX90640_1_HZ, 1.0f},   {"2 Hz", MLX90640_2_HZ, 2.0f},
    {"4 Hz", MLX90640_4_HZ, 4.0f},     {"8 Hz", MLX90640_8_HZ, 8.0f},   {"16 Hz", MLX90640_16_HZ, 16.0f},
    {"32 Hz", MLX90640_32_HZ, 32.0f},  {"64 Hz", MLX90640_64_HZ, 64.0f},
};
static const int RATE_COUNT = sizeof(RATES) / sizeof(RATES[0]);

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static Adafruit_MLX90640 mlx;
static ThermalImage image;
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
static Preferences prefs;

static float frame[MLX_W * MLX_H];

static String deviceId;
static String hostname;
static String topicBase;
static String topicImage;
static String topicState;
static String topicAvailability;
static String topicRateState;
static String topicRateCommand;
static String topicHaStatus;

static int rateIndex = 4;  // "8 Hz"
static bool sensorOk = false;
static int sensorErrors = 0;
static uint32_t lastSensorRetry = 0;
static uint32_t lastMqttAttempt = 0;
static uint32_t mqttRetryDelay = MQTT_RETRY_MIN_MS;
static bool discoveryPublished = false;
static bool discoveryPending = false;
static uint32_t mqttDrops = 0;
static uint32_t imagesSkipped = 0;
static uint32_t lastSocketWritable = 0;
static uint32_t lastStatePublish = 0;
static uint32_t lastImagePublish = 0;
static uint32_t lastFrameMs = 0;
static uint32_t frameCounter = 0;
static float fpsEstimate = 0;
static float lastMin = NAN;
static float lastMax = NAN;

// Forward declarations: the sensor helpers below publish through the MQTT
// helpers, which are defined further down.
static bool mqttPublishChecked(const String &topic, const uint8_t *payload, size_t len, bool retained,
                               const char *what);
static void mqttDrop(const char *why);
static bool publishDiscovery();
static void publishAvailability();

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static int rateIndexForHz(float hz) {
  for (int i = 0; i < RATE_COUNT; i++) {
    if (fabsf(RATES[i].hz - hz) < 0.01f) return i;
  }
  return 4;
}

static void buildIdentity() {
  String id = DEVICE_ID;
  if (id.length() == 0) {
    uint64_t mac = ESP.getEfuseMac();
    char buf[16];
    snprintf(buf, sizeof(buf), "%02x%02x%02x", (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
    id = String("thermalcam_") + buf;
  }
  deviceId = id;
  hostname = deviceId;
  hostname.replace('_', '-');

  topicBase = String(MQTT_BASE_TOPIC) + "/" + deviceId;
  topicImage = topicBase + "/image";
  topicState = topicBase + "/state";
  topicAvailability = topicBase + "/status";
  topicRateState = topicBase + "/refresh_rate";
  topicRateCommand = topicBase + "/refresh_rate/set";
  topicHaStatus = String(HA_DISCOVERY_PREFIX) + "/status";
}

// ---------------------------------------------------------------------------
// Sensor
// ---------------------------------------------------------------------------
static bool setupSensor() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);  // EEPROM download happens at a conservative speed
  if (!mlx.begin(MLX_I2C_ADDR, &Wire)) {
    Serial.println("[mlx] MLX90640 not found - check wiring and I2C address");
    return false;
  }
  Serial.printf("[mlx] found MLX90640, serial %04X%04X%04X\n", mlx.serialNumber[0], mlx.serialNumber[1],
                mlx.serialNumber[2]);
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(RATES[rateIndex].rate);
  Wire.setClock(I2C_CLOCK_HZ);
  Serial.printf("[mlx] refresh rate %s, I2C %lu Hz\n", RATES[rateIndex].label, (unsigned long)I2C_CLOCK_HZ);
  sensorErrors = 0;
  frameFilterReset();  // the offset pattern belongs to the old configuration
  return true;
}

static void publishRateState() {
  const char *label = RATES[rateIndex].label;
  mqttPublishChecked(topicRateState, (const uint8_t *)label, strlen(label), true, "refresh rate state");
}

static void applyRefreshRate(int idx, bool persist) {
  if (idx < 0 || idx >= RATE_COUNT) return;
  rateIndex = idx;
  if (sensorOk) {
    mlx.setRefreshRate(RATES[idx].rate);
  }
  if (persist) prefs.putUChar("rate", (uint8_t)idx);
  fpsEstimate = 0;
  frameFilterReset();  // integration time changed, so the offsets have too
  Serial.printf("[mlx] refresh rate set to %s\n", RATES[idx].label);
  publishRateState();
}

// Accepts "8 Hz", "8", "8.0", "0.5 Hz" ...
static void handleRateCommand(const String &payload) {
  String p = payload;
  p.trim();
  for (int i = 0; i < RATE_COUNT; i++) {
    if (p.equalsIgnoreCase(RATES[i].label)) {
      applyRefreshRate(i, true);
      return;
    }
  }
  float hz = p.toFloat();
  if (hz > 0) {
    for (int i = 0; i < RATE_COUNT; i++) {
      if (fabsf(RATES[i].hz - hz) < 0.01f) {
        applyRefreshRate(i, true);
        return;
      }
    }
  }
  Serial.printf("[mqtt] ignored unknown refresh rate '%s'\n", p.c_str());
  publishRateState();  // re-assert the real state so HA's select snaps back
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str());
  WiFi.setSleep(false);  // lower latency for streaming
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[wifi] connecting to %s", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] connected, IP %s, hostname %s.local\n", WiFi.localIP().toString().c_str(),
                  hostname.c_str());
  } else {
    Serial.println("[wifi] not connected yet, will keep retrying in the background");
  }
}

static void maintainWifi() {
  static uint32_t downSince = 0;
  static uint32_t lastKick = 0;

  if (WiFi.status() == WL_CONNECTED) {
    if (downSince) {
      Serial.printf("[wifi] link back up, IP %s\n", WiFi.localIP().toString().c_str());
      downSince = 0;
    }
    return;
  }

  const uint32_t now = millis();
  if (!downSince) {
    downSince = now;
    lastKick = now;
    Serial.println("[wifi] link down, letting auto-reconnect work");
    return;
  }

  // WiFi.reconnect() calls esp_wifi_disconnect() first, which aborts whatever
  // association attempt setAutoReconnect() already has in flight. Kicking it
  // every few seconds therefore keeps restarting the handshake instead of
  // letting one finish. Step in only once the built-in retries have had time to
  // fail, and then no more than once per grace period.
  if (now - downSince < WIFI_RECONNECT_GRACE_MS) return;
  if (now - lastKick < WIFI_RECONNECT_GRACE_MS) return;
  lastKick = now;
  Serial.println("[wifi] auto-reconnect has not recovered, forcing a reconnect");
  WiFi.reconnect();
}

// ---------------------------------------------------------------------------
// MQTT socket safety
// ---------------------------------------------------------------------------
// True when the TCP send buffer has room right now.
//
// WiFiClient::write() waits up to ten seconds (ten one-second select() retries)
// for space before giving up and returning a short count, and it waits on the
// calling task. Starting a publish on a socket that is already backed up
// therefore stalls the sensor loop and then truncates the packet, so check
// first and skip the frame instead.
static bool mqttSocketWritable() {
  const int sock = wifiClient.fd();
  if (sock < 0) return false;
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(sock, &wfds);
  struct timeval tv = {0, 0};
  return select(sock + 1, nullptr, &wfds, nullptr, &tv) > 0 && FD_ISSET(sock, &wfds);
}

// Closes the MQTT socket without writing anything to it.
//
// An MQTT packet declares its length up front. If only part of one reaches the
// broker, the broker keeps consuming everything that follows as the missing
// remainder: later PUBLISH headers and keepalive pings all disappear into that
// unfinished payload, until some byte in the middle of a JPEG is finally read
// as a control packet type and the broker resets the connection. Sending a
// DISCONNECT would only add more bytes to the phantom payload, so drop the TCP
// connection and start a fresh session instead.
static void mqttDrop(const char *why) {
  Serial.printf("[mqtt] dropping connection: %s\n", why);
  wifiClient.stop();
  (void)mqtt.connected();  // lets PubSubClient notice the loss and reset its state
  mqttDrops++;
  lastMqttAttempt = millis();
  lastSocketWritable = 0;
}

// Whether an image can be published right now, and how long the socket has been
// refusing data. A socket that never drains would otherwise sit there until the
// keepalive finally expires, publishing nothing the whole time.
static bool mqttReadyForImage() {
  const uint32_t now = millis();
  if (mqttSocketWritable()) {
    lastSocketWritable = now;
    return true;
  }
  if (!lastSocketWritable) {
    lastSocketWritable = now;
    return false;
  }
  if (now - lastSocketWritable >= (uint32_t)MQTT_SOCKET_STALL_MS) {
    mqttDrop("send buffer full for too long");
  }
  return false;
}

// Publishes a small message, treating a failed write as a corrupted stream.
//
// PubSubClient::publish() sends the whole packet in a single WiFiClient::write()
// and returns false when not all of it went out, which leaves the broker
// mid-packet exactly as a short image write does.
static bool mqttPublishChecked(const String &topic, const uint8_t *payload, size_t len, bool retained,
                               const char *what) {
  if (!mqtt.connected()) return false;
  if (len + topic.length() + 8 > MQTT_BUFFER_BYTES) {
    // Our own buffer limit, not a socket problem: the connection is still fine.
    Serial.printf("[mqtt] %s does not fit the %u byte buffer (%u bytes)\n", what, (unsigned)MQTT_BUFFER_BYTES,
                  (unsigned)len);
    return false;
  }
  if (mqtt.publish(topic.c_str(), payload, len, retained)) return true;
  mqttDrop(what);
  return false;
}

// ---------------------------------------------------------------------------
// MQTT + Home Assistant discovery
// ---------------------------------------------------------------------------
static void addDeviceInfo(JsonDocument &doc) {
  JsonObject dev = doc["device"].to<JsonObject>();
  dev["identifiers"].add(deviceId);
  dev["name"] = DEVICE_NAME;
  dev["manufacturer"] = "Seeed Studio / Melexis";
  dev["model"] = "XIAO ESP32S3 + MLX90640";
  dev["sw_version"] = FIRMWARE_VERSION;
#if ENABLE_HTTP_STREAM
  dev["configuration_url"] = "http://" + WiFi.localIP().toString() + "/";
#endif
  doc["availability_topic"] = topicAvailability;
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
}

static bool publishDiscoveryDoc(const String &component, const String &objectId, JsonDocument &doc) {
  String topic = String(HA_DISCOVERY_PREFIX) + "/" + component + "/" + deviceId + "/" + objectId + "/config";
  String payload;
  serializeJson(doc, payload);
  bool ok = mqttPublishChecked(topic, (const uint8_t *)payload.c_str(), payload.length(), true, "discovery config");
  if (!ok) Serial.printf("[mqtt] discovery publish failed for %s (%u bytes)\n", topic.c_str(), payload.length());
  return ok;
}

static void publishAvailability() {
  mqttPublishChecked(topicAvailability, (const uint8_t *)"online", 6, true, "availability");
}

// Stops at the first failure so a broken socket is not handed three more
// payloads that cannot go out either.
static bool publishDiscovery() {
  {
    JsonDocument doc;
    doc["name"] = "Thermal Image";
    doc["unique_id"] = deviceId + "_image";
    doc["topic"] = topicImage;
    doc["icon"] = "mdi:thermometer-lines";
    addDeviceInfo(doc);
    if (!publishDiscoveryDoc("camera", "image", doc)) return false;
  }
  {
    JsonDocument doc;
    doc["name"] = "Maximum Temperature";
    doc["unique_id"] = deviceId + "_max";
    doc["state_topic"] = topicState;
    doc["value_template"] = "{{ value_json.max }}";
    doc["unit_of_measurement"] = "°C";
    doc["device_class"] = "temperature";
    doc["state_class"] = "measurement";
    doc["suggested_display_precision"] = 1;
    addDeviceInfo(doc);
    if (!publishDiscoveryDoc("sensor", "max", doc)) return false;
  }
  {
    JsonDocument doc;
    doc["name"] = "Minimum Temperature";
    doc["unique_id"] = deviceId + "_min";
    doc["state_topic"] = topicState;
    doc["value_template"] = "{{ value_json.min }}";
    doc["unit_of_measurement"] = "°C";
    doc["device_class"] = "temperature";
    doc["state_class"] = "measurement";
    doc["suggested_display_precision"] = 1;
    addDeviceInfo(doc);
    if (!publishDiscoveryDoc("sensor", "min", doc)) return false;
  }
  {
    JsonDocument doc;
    doc["name"] = "Frame Rate";
    doc["unique_id"] = deviceId + "_refresh_rate";
    doc["command_topic"] = topicRateCommand;
    doc["state_topic"] = topicRateState;
    JsonArray options = doc["options"].to<JsonArray>();
    for (int i = 0; i < RATE_COUNT; i++) options.add(RATES[i].label);
    doc["entity_category"] = "config";
    doc["icon"] = "mdi:speedometer";
    addDeviceInfo(doc);
    if (!publishDiscoveryDoc("select", "refresh_rate", doc)) return false;
  }
  Serial.println("[mqtt] discovery published");
  return true;
}

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (topicRateCommand.equals(topic)) {
    handleRateCommand(msg);
  } else if (topicHaStatus.equals(topic)) {
    // Home Assistant (re)started: announce ourselves again so entities are recreated.
    // Publishing from inside the callback would reuse the PubSubClient buffer that
    // still holds this incoming packet, so let the main loop do it.
    if (msg == "online") {
      Serial.println("[mqtt] Home Assistant came online, re-announcing");
      discoveryPending = true;
    }
  }
}

static void mqttConnect() {
  Serial.printf("[mqtt] connecting to %s:%d as %s... ", MQTT_HOST, MQTT_PORT, deviceId.c_str());
  const char *user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
  const char *pass = strlen(MQTT_USER) ? MQTT_PASSWORD : nullptr;
  bool ok = mqtt.connect(deviceId.c_str(), user, pass, topicAvailability.c_str(), 1, true, "offline");
  if (!ok) {
    Serial.printf("failed, rc=%d\n", mqtt.state());
    wifiClient.stop();  // make sure a half-open socket is not left behind
    mqttRetryDelay = min<uint32_t>(mqttRetryDelay * 2, (uint32_t)MQTT_RETRY_MAX_MS);
    return;
  }
  Serial.println("connected");
  mqttRetryDelay = MQTT_RETRY_MIN_MS;
  lastSocketWritable = millis();
  mqtt.subscribe(topicRateCommand.c_str());
  mqtt.subscribe(topicHaStatus.c_str());
  // The discovery configs are retained, so the broker keeps serving them to
  // Home Assistant on its own. Re-sending them on every reconnect makes Home
  // Assistant tear down and rebuild all four entities each time, which is a
  // large part of why the dashboard felt slow.
  if (!discoveryPublished) discoveryPending = true;
  publishAvailability();
  publishRateState();
}

static void maintainMqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    // The socket cannot survive the link going away; close it now so the next
    // connect starts from a clean file descriptor.
    if (mqtt.connected()) mqttDrop("wifi link lost");
    return;
  }
  if (mqtt.connected()) {
    mqtt.loop();
    if (discoveryPending && mqtt.connected()) {
      discoveryPending = false;
      if (publishDiscovery()) {
        discoveryPublished = true;
        publishAvailability();
        publishRateState();
      }
    }
    return;
  }
  if (millis() - lastMqttAttempt < mqttRetryDelay) return;
  lastMqttAttempt = millis();
  mqttConnect();
}

static void publishState() {
  JsonDocument doc;
  doc["min"] = roundf(lastMin * 10.0f) / 10.0f;
  doc["max"] = roundf(lastMax * 10.0f) / 10.0f;
  doc["fps"] = roundf(fpsEstimate * 10.0f) / 10.0f;
  doc["refresh_rate"] = RATES[rateIndex].label;
  doc["frames"] = frameCounter;
  char buf[192];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqttPublishChecked(topicState, (const uint8_t *)buf, n, true, "state");
}

// beginPublish() tells the broker how many payload bytes to expect, so once the
// header is out the whole JPEG has to follow. Writing it in chunks means a
// backed-up socket is detected at the first stalled chunk rather than after the
// full ten second retry budget, and any shortfall costs one clean reconnect
// instead of a silently desynchronised session.
static void publishImage(const uint8_t *jpeg, size_t len) {
  if (!mqtt.beginPublish(topicImage.c_str(), len, MQTT_IMAGE_RETAIN)) {
    mqttDrop("image header write failed");
    return;
  }

  size_t sent = 0;
  while (sent < len) {
    const size_t chunk = min<size_t>(len - sent, (size_t)MQTT_WRITE_CHUNK_BYTES);
    const size_t n = mqtt.write(jpeg + sent, chunk);
    sent += n;
    if (n < chunk) break;  // send buffer is backed up; this packet is now truncated
  }
  mqtt.endPublish();

  if (sent != len) {
    Serial.printf("[mqtt] image truncated at %u/%u bytes\n", (unsigned)sent, (unsigned)len);
    mqttDrop("truncated image publish");
  }
}

// ---------------------------------------------------------------------------
// Frame pipeline
// ---------------------------------------------------------------------------
static void captureAndPublish() {
  int status = mlx.getFrame(frame);
  if (status != 0) {
    sensorErrors++;
    Serial.printf("[mlx] getFrame failed (%d), consecutive errors %d\n", status, sensorErrors);
    if (sensorErrors >= 10) {
      Serial.println("[mlx] too many errors, re-initialising sensor");
      sensorOk = setupSensor();
    }
    delay(50);
    return;
  }
  sensorErrors = 0;

  // Before anything reads these temperatures: the raw frame carries the
  // sensor's own period-2 offset pattern and the occasional pixel that is
  // tens of degrees out, and both would otherwise reach the palette and the
  // min/max sensors.
  frameFilterApply(frame);

  float lo, hi;
  if (!ThermalImage::stats(frame, lo, hi)) {
    Serial.println("[mlx] frame contained no valid pixels");
    return;
  }
  lastMin = lo;
  lastMax = hi;

  const uint32_t now = millis();
  if (lastFrameMs) {
    const float inst = 1000.0f / (float)max<uint32_t>(now - lastFrameMs, 1);
    fpsEstimate = fpsEstimate <= 0 ? inst : fpsEstimate * 0.8f + inst * 0.2f;
  }
  lastFrameMs = now;
  frameCounter++;

  // Colour mapping range
  float rlo, rhi;
#if AUTO_RANGE
  if (!ThermalImage::range(frame, RANGE_LOW_PERCENTILE, RANGE_HIGH_PERCENTILE, rlo, rhi)) {
    rlo = lo;
    rhi = hi;
  }
  if (rhi - rlo < MIN_RANGE_SPAN_C) {
    const float mid = (rlo + rhi) * 0.5f;
    rlo = mid - MIN_RANGE_SPAN_C * 0.5f;
    rhi = mid + MIN_RANGE_SPAN_C * 0.5f;
  }
  // Ease towards the new range instead of snapping to it, so someone
  // walking through the scene does not recolour the whole background from
  // one frame to the next.
  static float smoothLo = NAN, smoothHi = NAN;
  if (isnan(smoothLo) || isnan(smoothHi)) {
    smoothLo = rlo;
    smoothHi = rhi;
  } else {
    smoothLo += (rlo - smoothLo) * RANGE_ADAPT_RATE;
    smoothHi += (rhi - smoothHi) * RANGE_ADAPT_RATE;
  }
  rlo = smoothLo;
  rhi = smoothHi;
#else
  rlo = FIXED_RANGE_MIN_C;
  rhi = FIXED_RANGE_MAX_C;
#endif

  image.render(frame, rlo, rhi);

  uint8_t *jpeg = nullptr;
  size_t jpegLen = 0;
  if (!fmt2jpg((uint8_t *)image.pixels(), image.pixelBytes(), image.width(), image.height(), PIXFORMAT_RGB888,
               JPEG_QUALITY, &jpeg, &jpegLen)) {
    Serial.println("[img] JPEG encode failed");
    return;
  }

  streamServerSetFrame(jpeg, jpegLen);
  {
    StreamStats s;
    s.minC = lo;
    s.maxC = hi;
    s.fps = fpsEstimate;
    strlcpy(s.rate, RATES[rateIndex].label, sizeof(s.rate));
    streamServerSetStats(s);
  }

  if (mqtt.connected()) {
    if (MQTT_IMAGE_INTERVAL_MS == 0 || now - lastImagePublish >= (uint32_t)MQTT_IMAGE_INTERVAL_MS) {
      if (mqttReadyForImage()) {
        lastImagePublish = now;
        publishImage(jpeg, jpegLen);
      } else {
        // Dropping one image is far cheaper than blocking the sensor loop for
        // seconds and then truncating the packet.
        imagesSkipped++;
      }
    }
    if (mqtt.connected() && now - lastStatePublish >= (uint32_t)STATE_PUBLISH_INTERVAL_MS) {
      lastStatePublish = now;
      publishState();
    }
  }

  free(jpeg);

  static uint32_t lastLog = 0;
  if (now - lastLog > 5000) {
    lastLog = now;
    Serial.printf(
        "[frame] #%lu min %.1f C max %.1f C  %.1f fps  jpeg %u B  heap %u  stream clients %d  mqtt %s  drops %lu  "
        "images skipped %lu\n",
        (unsigned long)frameCounter, lo, hi, fpsEstimate, (unsigned)jpegLen, (unsigned)ESP.getFreeHeap(),
        streamServerClientCount(), mqtt.connected() ? "up" : "down", (unsigned long)mqttDrops,
        (unsigned long)imagesSkipped);

    // What the pattern filter is taking out. These should settle within a few
    // frames and then barely move; all zeroes means it never ran, and numbers
    // that keep swinging mean it is chasing the scene rather than the sensor.
    FrameFilterStats fs;
    frameFilterGetStats(fs);
    Serial.printf("[fpn]   %s  rows %.3f C  cols [%+.3f %+.3f %+.3f %+.3f] C  subpage %+.3f C\n",
                  fs.primed ? "locked" : "priming", fs.rowAmp, fs.colCycle[0], fs.colCycle[1], fs.colCycle[2],
                  fs.colCycle[3], fs.chess);
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n[boot] MLX90640 thermal camera firmware %s\n", FIRMWARE_VERSION);
  Serial.printf("[boot] PSRAM %s, free heap %u\n", psramFound() ? "available" : "NOT found", (unsigned)ESP.getFreeHeap());

  buildIdentity();
  Serial.printf("[boot] device id %s\n", deviceId.c_str());

  prefs.begin("thermalcam", false);
  rateIndex = prefs.getUChar("rate", (uint8_t)rateIndexForHz(DEFAULT_REFRESH_RATE_HZ));
  if (rateIndex < 0 || rateIndex >= RATE_COUNT) rateIndex = rateIndexForHz(DEFAULT_REFRESH_RATE_HZ);

  if (!image.begin()) {
    Serial.println("[boot] failed to allocate image buffer");
  }

  sensorOk = setupSensor();

  setupWifi();

  if (MDNS.begin(hostname.c_str())) {
    MDNS.addService("http", "tcp", HTTP_PORT);
  }
  streamServerBegin(HTTP_PORT);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(MQTT_BUFFER_BYTES);  // discovery payloads; images use the streaming publish API
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
}

void loop() {
  maintainWifi();
  maintainMqtt();

  if (sensorOk) {
    captureAndPublish();
  } else {
    if (millis() - lastSensorRetry > 5000) {
      lastSensorRetry = millis();
      sensorOk = setupSensor();
    }
    delay(100);
  }
}
