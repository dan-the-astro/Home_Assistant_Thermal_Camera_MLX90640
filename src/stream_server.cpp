#include "stream_server.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include "config.h"

#if ENABLE_HTTP_STREAM

namespace {

constexpr int kMaxStreamClients = 4;
constexpr uint32_t kRequestTimeoutMs = 1500;
// A single WiFiClient::write() already blocks for up to ten seconds on a client
// that is not draining its socket. Without an overall deadline a stalled viewer
// can hold the server task for minutes and freeze the other viewers with it.
constexpr uint32_t kWriteTimeoutMs = 4000;
constexpr const char *kBoundary = "thermalframe";

WiFiServer *server = nullptr;
SemaphoreHandle_t mutex = nullptr;

// Shared latest frame (written by the sensor loop, read by the server task).
uint8_t *sharedBuf = nullptr;
size_t sharedCap = 0;
size_t sharedLen = 0;
uint32_t sharedSeq = 0;
StreamStats sharedStats;

// Private copy used while writing to sockets so the sensor loop is never blocked by a slow client.
uint8_t *sendBuf = nullptr;
size_t sendCap = 0;

struct StreamClient {
  WiFiClient client;
  uint32_t lastSeq = 0;
  bool active = false;
};
StreamClient streams[kMaxStreamClients];

const char kIndexHtml[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MLX90640 Thermal Camera</title>
<style>
body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:1rem;text-align:center}
img{width:100%;max-width:640px;border-radius:6px;background:#000}
.s{margin-top:.6rem;font-size:1.05rem;color:#ccc}
a{color:#8cf}
</style></head>
<body>
<h2>MLX90640 Thermal Camera</h2>
<img src="/stream" alt="thermal stream">
<div class="s" id="s">waiting for data&hellip;</div>
<p class="s"><a href="/snapshot.jpg">snapshot.jpg</a> &middot; <a href="/state.json">state.json</a></p>
<script>
setInterval(async function(){try{var j=await (await fetch('/state.json')).json();
document.getElementById('s').textContent='min '+j.min.toFixed(1)+' °C · max '+j.max.toFixed(1)+' °C · '+j.fps.toFixed(1)+' fps · sensor '+j.rate;}catch(e){}},1000);
</script>
</body></html>
)HTML";

void *bigAlloc(size_t n) {
  void *p = nullptr;
  if (psramFound()) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = malloc(n);
  return p;
}

bool ensureCapacity(uint8_t *&buf, size_t &cap, size_t need) {
  if (need <= cap) return true;
  size_t newCap = need + need / 4 + 1024;
  uint8_t *p = (uint8_t *)bigAlloc(newCap);
  if (!p) return false;
  if (buf) free(buf);
  buf = p;
  cap = newCap;
  return true;
}

// Copies the latest shared frame into sendBuf. Returns its length (0 if none yet).
size_t copyLatest(uint32_t &seqOut) {
  size_t len = 0;
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (sharedLen && ensureCapacity(sendBuf, sendCap, sharedLen)) {
    memcpy(sendBuf, sharedBuf, sharedLen);
    len = sharedLen;
    seqOut = sharedSeq;
  }
  xSemaphoreGive(mutex);
  return len;
}

StreamStats copyStats() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  StreamStats s = sharedStats;
  xSemaphoreGive(mutex);
  return s;
}

bool writeAll(WiFiClient &c, const uint8_t *data, size_t len) {
  size_t sent = 0;
  const uint32_t deadline = millis() + kWriteTimeoutMs;
  while (sent < len && c.connected()) {
    size_t n = c.write(data + sent, len - sent);
    if (n == 0) return false;
    sent += n;
    // A short write leaves this multipart part truncated, so the caller has to
    // drop the client rather than start the next boundary on a desynced stream.
    if (sent < len && (int32_t)(millis() - deadline) >= 0) return false;
  }
  return sent == len;
}

bool writeStr(WiFiClient &c, const char *s) { return writeAll(c, (const uint8_t *)s, strlen(s)); }

void sendSimple(WiFiClient &c, const char *status, const char *type, const char *body) {
  char hdr[160];
  snprintf(hdr, sizeof(hdr),
           "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %u\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
           status, type, (unsigned)strlen(body));
  writeStr(c, hdr);
  writeStr(c, body);
}

bool sendFramePart(WiFiClient &c, const uint8_t *jpeg, size_t len) {
  char hdr[128];
  snprintf(hdr, sizeof(hdr), "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", kBoundary,
           (unsigned)len);
  if (!writeStr(c, hdr)) return false;
  if (!writeAll(c, jpeg, len)) return false;
  return writeStr(c, "\r\n");
}

// Reads the request head (up to the blank line) with a deadline. Returns the request path.
String readRequestPath(WiFiClient &c) {
  String head;
  head.reserve(512);
  const uint32_t deadline = millis() + kRequestTimeoutMs;
  bool complete = false;
  while (!complete && c.connected() && (int32_t)(deadline - millis()) > 0) {
    while (c.available()) {
      char ch = (char)c.read();
      if (head.length() < 2048) head += ch;
      if (head.endsWith("\r\n\r\n")) {
        complete = true;
        break;
      }
    }
    if (!complete) vTaskDelay(pdMS_TO_TICKS(2));
  }
  int sp1 = head.indexOf(' ');
  if (sp1 < 0) return "";
  int sp2 = head.indexOf(' ', sp1 + 1);
  if (sp2 < 0) return "";
  String path = head.substring(sp1 + 1, sp2);
  int q = path.indexOf('?');
  if (q >= 0) path = path.substring(0, q);
  return path;
}

void handleNewClient(WiFiClient c) {
  String path = readRequestPath(c);
  if (path.length() == 0) {
    c.stop();
    return;
  }

  if (path == "/stream") {
    int slot = -1;
    for (int i = 0; i < kMaxStreamClients; i++) {
      if (!streams[i].active) {
        slot = i;
        break;
      }
    }
    if (slot < 0) {
      sendSimple(c, "503 Service Unavailable", "text/plain", "Too many stream clients\n");
      c.stop();
      return;
    }
    char hdr[200];
    snprintf(hdr, sizeof(hdr),
             "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=%s\r\n"
             "Cache-Control: no-cache\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n",
             kBoundary);
    if (!writeStr(c, hdr)) {
      c.stop();
      return;
    }
    streams[slot].client = c;
    streams[slot].lastSeq = 0;
    streams[slot].active = true;
    return;  // kept open; frames are pushed from the task loop
  }

  if (path == "/snapshot.jpg") {
    uint32_t seq = 0;
    size_t len = copyLatest(seq);
    if (!len) {
      sendSimple(c, "503 Service Unavailable", "text/plain", "No frame captured yet\n");
    } else {
      char hdr[160];
      snprintf(hdr, sizeof(hdr),
               "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
               (unsigned)len);
      writeStr(c, hdr);
      writeAll(c, sendBuf, len);
    }
    c.stop();
    return;
  }

  if (path == "/state.json") {
    StreamStats s = copyStats();
    char body[160];
    snprintf(body, sizeof(body), "{\"min\":%.2f,\"max\":%.2f,\"fps\":%.2f,\"rate\":\"%s\",\"clients\":%d}", s.minC,
             s.maxC, s.fps, s.rate, streamServerClientCount());
    sendSimple(c, "200 OK", "application/json", body);
    c.stop();
    return;
  }

  if (path == "/" || path == "/index.html") {
    const size_t len = strlen_P(kIndexHtml);
    char hdr[160];
    snprintf(hdr, sizeof(hdr),
             "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: %u\r\nConnection: close\r\n\r\n",
             (unsigned)len);
    writeStr(c, hdr);
    writeAll(c, (const uint8_t *)kIndexHtml, len);
    c.stop();
    return;
  }

  sendSimple(c, "404 Not Found", "text/plain", "Not found\n");
  c.stop();
}

void serviceStreams() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  const uint32_t seq = sharedSeq;
  xSemaphoreGive(mutex);

  bool copied = false;
  size_t len = 0;
  uint32_t copiedSeq = 0;

  for (int i = 0; i < kMaxStreamClients; i++) {
    StreamClient &s = streams[i];
    if (!s.active) continue;
    if (!s.client.connected()) {
      s.client.stop();
      s.active = false;
      continue;
    }
    if (seq == 0 || s.lastSeq == seq) continue;
    if (!copied) {
      len = copyLatest(copiedSeq);
      copied = true;
    }
    if (!len) continue;
    if (!sendFramePart(s.client, sendBuf, len)) {
      s.client.stop();
      s.active = false;
      continue;
    }
    s.lastSeq = copiedSeq;
  }
}

void serverTask(void *) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient c = server->available();
      if (c) {
        c.setNoDelay(true);
        handleNewClient(c);
      }
      serviceStreams();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace

void streamServerBegin(uint16_t port) {
  if (server) return;
  mutex = xSemaphoreCreateMutex();
  server = new WiFiServer(port);
  server->begin();
  server->setNoDelay(true);
  // Core 0 keeps the socket work away from the sensor / MQTT loop on core 1.
  xTaskCreatePinnedToCore(serverTask, "mjpeg", 8192, nullptr, 1, nullptr, 0);
}

void streamServerSetFrame(const uint8_t *jpeg, size_t len) {
  if (!mutex || !jpeg || !len) return;
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (ensureCapacity(sharedBuf, sharedCap, len)) {
    memcpy(sharedBuf, jpeg, len);
    sharedLen = len;
    sharedSeq++;
    if (sharedSeq == 0) sharedSeq = 1;
  }
  xSemaphoreGive(mutex);
}

void streamServerSetStats(const StreamStats &stats) {
  if (!mutex) return;
  xSemaphoreTake(mutex, portMAX_DELAY);
  sharedStats = stats;
  xSemaphoreGive(mutex);
}

int streamServerClientCount() {
  int n = 0;
  for (int i = 0; i < kMaxStreamClients; i++) n += streams[i].active ? 1 : 0;
  return n;
}

#else  // ENABLE_HTTP_STREAM == 0

void streamServerBegin(uint16_t) {}
void streamServerSetFrame(const uint8_t *, size_t) {}
void streamServerSetStats(const StreamStats &) {}
int streamServerClientCount() { return 0; }

#endif
