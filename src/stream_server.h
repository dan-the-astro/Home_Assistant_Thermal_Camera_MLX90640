// Small HTTP server running in its own FreeRTOS task:
//   GET /             status page with the live stream embedded
//   GET /stream       multipart MJPEG stream (use with the HA "MJPEG IP Camera" integration)
//   GET /snapshot.jpg latest frame as a single JPEG
//   GET /state.json   {"min":..,"max":..,"fps":..,"rate":".."}
#pragma once

#include <stddef.h>
#include <stdint.h>

struct StreamStats {
  float minC = 0;
  float maxC = 0;
  float fps = 0;
  char rate[16] = "";
};

void streamServerBegin(uint16_t port);
// Copies the JPEG into the shared frame buffer and wakes any stream clients.
void streamServerSetFrame(const uint8_t *jpeg, size_t len);
void streamServerSetStats(const StreamStats &stats);
int streamServerClientCount();
