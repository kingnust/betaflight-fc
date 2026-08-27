#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_ENABLE_DIRECT_WIFI_RC)

#include "DroneProtoDirectRc.hpp"
#include "DroneProtoDirectRcProtocol.hpp"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

#ifndef ESPFC_DRONE_PROTO_DIRECT_RC_WIFI_CHANNEL
#define ESPFC_DRONE_PROTO_DIRECT_RC_WIFI_CHANNEL 6
#endif

#ifndef ESPFC_DRONE_PROTO_DIRECT_RC_TIMEOUT_MS
#define ESPFC_DRONE_PROTO_DIRECT_RC_TIMEOUT_MS 350
#endif

#ifndef ESPFC_DRONE_PROTO_DIRECT_RC_LINK_ID
#define ESPFC_DRONE_PROTO_DIRECT_RC_LINK_ID 0x6D5A31C7UL
#endif

namespace Espfc::Device {
namespace {

namespace Protocol = DirectRcProtocol;
using PacketMode = Protocol::Mode;
using DirectRcPacket = Protocol::Packet;
constexpr size_t kPacketChannels = Protocol::CHANNEL_COUNT;
constexpr uint16_t kMinUs = Protocol::CHANNEL_MIN_US;
constexpr uint16_t kMidUs = Protocol::CHANNEL_MID_US;

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
bool s_ready = false;
int32_t s_initError = 0;
PacketMode s_mode = PacketMode::NONE;
bool s_newFrame = false;
uint16_t s_channels[kPacketChannels] = {};
uint32_t s_lastMs = 0;
uint16_t s_lastSeq = 0;
uint32_t s_received = 0;
uint32_t s_valid = 0;
uint32_t s_badCrc = 0;
uint32_t s_badLink = 0;
uint32_t s_badSource = 0;
uint32_t s_badSize = 0;
uint32_t s_badValue = 0;
uint32_t s_duplicate = 0;
uint32_t s_outOfOrder = 0;
uint32_t s_missed = 0;
bool s_hasSeq = false;
uint8_t s_lastSourceMac[6] = {};
bool s_hasSourceMac = false;

void setSafeChannels()
{
  for(size_t i = 0; i < kPacketChannels; ++i)
  {
    s_channels[i] = kMidUs;
  }
  s_channels[2] = kMinUs;
}

bool isFreshTimestamp(uint32_t nowMs, uint32_t lastMs)
{
  if(lastMs == 0) return false;
  const int32_t age = static_cast<int32_t>(nowMs - lastMs);
  // The ESP-NOW callback runs on the Wi-Fi task. It can publish a timestamp
  // just after another core captured `nowMs`, so a small negative age is
  // fresh, not a 49-day-old packet caused by unsigned underflow.
  return age < 0
    ? (lastMs - nowMs) <= ESPFC_DRONE_PROTO_DIRECT_RC_TIMEOUT_MS
    : static_cast<uint32_t>(age) <= ESPFC_DRONE_PROTO_DIRECT_RC_TIMEOUT_MS;
}

uint32_t timestampAgeMs(uint32_t nowMs, uint32_t lastMs)
{
  if(lastMs == 0) return 0;
  const int32_t age = static_cast<int32_t>(nowMs - lastMs);
  return age < 0 ? 0u : static_cast<uint32_t>(age);
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
  portENTER_CRITICAL_ISR(&s_mux);
  ++s_received;
  if(mac != nullptr)
  {
    std::memcpy(s_lastSourceMac, mac, sizeof(s_lastSourceMac));
    s_hasSourceMac = true;
  }
  portEXIT_CRITICAL_ISR(&s_mux);

  if(!Protocol::trustedSourceMac(mac))
  {
    portENTER_CRITICAL_ISR(&s_mux);
    ++s_badSource;
    portEXIT_CRITICAL_ISR(&s_mux);
    return;
  }

  DirectRcPacket packet;
  PacketMode packetMode = PacketMode::NONE;
  const Protocol::DecodeResult decodeResult = Protocol::decode(
    data, len < 0 ? 0u : static_cast<size_t>(len),
    ESPFC_DRONE_PROTO_DIRECT_RC_LINK_ID, packet, packetMode);
  if(decodeResult != Protocol::DecodeResult::ACCEPTED)
  {
    portENTER_CRITICAL_ISR(&s_mux);
    switch(decodeResult)
    {
      case Protocol::DecodeResult::BAD_SIZE: ++s_badSize; break;
      case Protocol::DecodeResult::BAD_LINK: ++s_badLink; break;
      case Protocol::DecodeResult::BAD_VALUE: ++s_badValue; break;
      case Protocol::DecodeResult::BAD_CRC_OR_HEADER: ++s_badCrc; break;
      case Protocol::DecodeResult::ACCEPTED: break;
    }
    portEXIT_CRITICAL_ISR(&s_mux);
    return;
  }

  const uint32_t nowMs = millis();
  portENTER_CRITICAL_ISR(&s_mux);
  // Once the link has timed out, accept the next valid packet as a new
  // sequence baseline. This lets a rebooted SuperMini reconnect immediately
  // even though its 16-bit sequence counter restarted at zero.
  const bool sequenceIsFresh = s_lastMs != 0 &&
    static_cast<uint32_t>(nowMs - s_lastMs) <= ESPFC_DRONE_PROTO_DIRECT_RC_TIMEOUT_MS;
  if(s_hasSeq && sequenceIsFresh)
  {
    const uint16_t delta = static_cast<uint16_t>(packet.sequence - s_lastSeq);
    if(delta == 0)
    {
      ++s_duplicate;
      portEXIT_CRITICAL_ISR(&s_mux);
      return;
    }
    if(delta > 32768)
    {
      ++s_outOfOrder;
      portEXIT_CRITICAL_ISR(&s_mux);
      return;
    }
    if(delta > 1)
    {
      s_missed += static_cast<uint32_t>(delta - 1);
    }
  }
  s_hasSeq = true;
  s_mode = packetMode;
  if(packetMode != PacketMode::NONE)
  {
    for(size_t i = 0; i < kPacketChannels; ++i)
    {
      s_channels[i] = packet.channels[i];
    }
    s_lastMs = nowMs;
    s_lastSeq = packet.sequence;
    s_newFrame = packetMode == PacketMode::DIRECT;
    ++s_valid;
  }
  else
  {
    s_mode = PacketMode::NONE;
    s_newFrame = false;
    setSafeChannels();
    s_lastMs = nowMs;
    s_lastSeq = packet.sequence;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

} // namespace

bool DroneProtoDirectRc::begin()
{
  s_ready = false;
  s_initError = 0;
  portENTER_CRITICAL(&s_mux);
  setSafeChannels();
  portEXIT_CRITICAL(&s_mux);

  WiFi.persistent(false);
  if(!WiFi.mode(WIFI_STA))
  {
    s_initError = -1;
    return false;
  }
  if(!WiFi.setSleep(false))
  {
    s_initError = -2;
    return false;
  }

  esp_err_t result = esp_wifi_set_ps(WIFI_PS_NONE);
  if(result != ESP_OK)
  {
    s_initError = result;
    return false;
  }
  result = esp_wifi_set_channel(ESPFC_DRONE_PROTO_DIRECT_RC_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if(result != ESP_OK)
  {
    s_initError = result;
    return false;
  }

  result = esp_now_init();
  if(result != ESP_OK)
  {
    s_initError = result;
    return false;
  }

  result = esp_now_register_recv_cb(onReceive);
  if(result != ESP_OK)
  {
    s_initError = result;
    esp_now_deinit();
    return false;
  }

  s_ready = true;
  return true;
}

bool DroneProtoDirectRc::consumeNewFrame()
{
  bool result = false;
  portENTER_CRITICAL(&s_mux);
  result = s_newFrame;
  s_newFrame = false;
  portEXIT_CRITICAL(&s_mux);
  return result;
}

bool DroneProtoDirectRc::active(uint32_t nowMs)
{
  PacketMode mode = PacketMode::NONE;
  uint32_t last = 0;
  portENTER_CRITICAL(&s_mux);
  mode = s_mode;
  last = s_lastMs;
  portEXIT_CRITICAL(&s_mux);
  return s_ready && mode == PacketMode::DIRECT && isFreshTimestamp(nowMs, last);
}

bool DroneProtoDirectRc::trainerSidebandActive(uint32_t nowMs)
{
  PacketMode mode = PacketMode::NONE;
  uint32_t last = 0;
  portENTER_CRITICAL(&s_mux);
  mode = s_mode;
  last = s_lastMs;
  portEXIT_CRITICAL(&s_mux);
  return s_ready && mode == PacketMode::TRAINER_SIDEBAND && isFreshTimestamp(nowMs, last);
}

void DroneProtoDirectRc::getChannels(uint16_t *data, size_t len)
{
  if(data == nullptr) return;

  portENTER_CRITICAL(&s_mux);
  for(size_t i = 0; i < len; ++i)
  {
    if(i < kPacketChannels)
    {
      data[i] = s_channels[i];
    }
    else
    {
      data[i] = kMidUs;
    }
  }
  portEXIT_CRITICAL(&s_mux);
}

bool DroneProtoDirectRc::ready() { return s_ready; }

int32_t DroneProtoDirectRc::initError() { return s_initError; }

uint32_t DroneProtoDirectRc::ageMs(uint32_t nowMs)
{
  uint32_t last = 0;
  portENTER_CRITICAL(&s_mux);
  last = s_lastMs;
  portEXIT_CRITICAL(&s_mux);
  return timestampAgeMs(nowMs, last);
}

uint32_t DroneProtoDirectRc::receivedFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_received;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::validFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_valid;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::badCrcFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_badCrc;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::badLinkFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_badLink;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::badSourceFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_badSource;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::badSizeFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_badSize;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::badValueFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_badValue;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::duplicateFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_duplicate;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::outOfOrderFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_outOfOrder;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint32_t DroneProtoDirectRc::missedFrames()
{
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_missed;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

uint16_t DroneProtoDirectRc::lastSequence()
{
  portENTER_CRITICAL(&s_mux);
  const uint16_t value = s_lastSeq;
  portEXIT_CRITICAL(&s_mux);
  return value;
}

bool DroneProtoDirectRc::lastSourceMac(uint8_t *data, size_t len)
{
  if(data == nullptr || len < sizeof(s_lastSourceMac)) return false;
  portENTER_CRITICAL(&s_mux);
  const bool available = s_hasSourceMac;
  if(available) std::memcpy(data, s_lastSourceMac, sizeof(s_lastSourceMac));
  portEXIT_CRITICAL(&s_mux);
  return available;
}

} // namespace Espfc::Device

#endif
