#pragma once

#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_ENABLE_DIRECT_WIFI_RC)

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

namespace Espfc::Device {

class DroneProtoDirectRc
{
public:
  static bool begin();
  static bool consumeNewFrame();
  static bool active(uint32_t nowMs = millis());
  static void getChannels(uint16_t *data, size_t len);

  static bool ready();
  static int32_t initError();
  static uint32_t ageMs(uint32_t nowMs = millis());
  static uint32_t receivedFrames();
  static uint32_t validFrames();
  static uint32_t badCrcFrames();
  static uint32_t badLinkFrames();
  static uint32_t badSizeFrames();
  static uint32_t badValueFrames();
  static uint32_t duplicateFrames();
  static uint32_t outOfOrderFrames();
  static uint32_t missedFrames();
  static uint16_t lastSequence();
};

} // namespace Espfc::Device

#endif
