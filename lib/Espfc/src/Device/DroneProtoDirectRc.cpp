#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_ENABLE_DIRECT_WIFI_RC)

#include "DroneProtoDirectRc.hpp"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

#ifndef ESPFC_DRONE_PROTO_DIRECT_RC_WIFI_CHANNEL
#define ESPFC_DRONE_PROTO_DIRECT_RC_WIFI_CHANNEL 6
#endif

#ifndef ESPFC_DRONE_PROTO_DIRECT_RC_TIMEOUT_MS
#define ESPFC_DRONE_PROTO_DIRECT_RC_TIMEOUT_MS 180
#endif

#ifndef ESPFC_DRONE_PROTO_DIRECT_RC_LINK_ID
#define ESPFC_DRONE_PROTO_DIRECT_RC_LINK_ID 0x6D5A31C7UL
#endif

namespace Espfc::Device {
namespace {

constexpr uint32_t kMagic = 0x31524344UL;  // "DRC1" little-endian
constexpr uint8_t kVersion = 2;
constexpr size_t kPacketChannels = 16;
constexpr uint16_t kMinUs = 988;
constexpr uint16_t kMidUs = 1500;
constexpr uint16_t kMaxUs = 2012;

enum class PacketMode : uint8_t
{
  NONE,
  TRAINER_SIDEBAND,
  DIRECT,
};

struct __attribute__((packed)) DirectRcPacket
{
  uint32_t magic;
  uint32_t linkId;
  uint8_t version;
  uint8_t channelCount;
  uint8_t flags;
  uint8_t reserved;
  uint16_t sequence;
  uint32_t timeMs;
  uint16_t channels[kPacketChannels];
  uint16_t crc;
};

static_assert(sizeof(DirectRcPacket) == 52, "Direct RC packet layout changed");

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
uint32_t s_badSize = 0;
uint32_t s_badValue = 0;
uint32_t s_duplicate = 0;
uint32_t s_outOfOrder = 0;
uint32_t s_missed = 0;
bool s_hasSeq = false;

uint16_t crc16Ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xffff;
  while(len--)
  {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for(uint8_t i = 0; i < 8; ++i)
    {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void setSafeChannels()
{
  for(size_t i = 0; i < kPacketChannels; ++i)
  {
    s_channels[i] = kMidUs;
  }
  s_channels[2] = kMinUs;
}

bool validChannelValues(const DirectRcPacket& packet)
{
  for(size_t i = 0; i < kPacketChannels; ++i)
  {
    if(packet.channels[i] < kMinUs || packet.channels[i] > kMaxUs)
    {
      return false;
    }
  }
  return true;
}

bool decodePacketMode(uint8_t flags, PacketMode& mode)
{
  switch(flags)
  {
    case 0x00: mode = PacketMode::NONE; return true;
    case 0x03: mode = PacketMode::DIRECT; return true;
    case 0x04: mode = PacketMode::TRAINER_SIDEBAND; return true;
    default: return false;
  }
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
  (void)mac;
  portENTER_CRITICAL_ISR(&s_mux);
  ++s_received;
  portEXIT_CRITICAL_ISR(&s_mux);

  if(data == nullptr || len != static_cast<int>(sizeof(DirectRcPacket)))
  {
    portENTER_CRITICAL_ISR(&s_mux);
    ++s_badSize;
    portEXIT_CRITICAL_ISR(&s_mux);
    return;
  }

  DirectRcPacket packet;
  std::memcpy(&packet, data, sizeof(packet));

  const uint16_t crc = crc16Ccitt(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - sizeof(packet.crc));
  if(crc != packet.crc || packet.magic != kMagic || packet.version != kVersion || packet.channelCount != kPacketChannels)
  {
    portENTER_CRITICAL_ISR(&s_mux);
    ++s_badCrc;
    portEXIT_CRITICAL_ISR(&s_mux);
    return;
  }

  if(packet.linkId != ESPFC_DRONE_PROTO_DIRECT_RC_LINK_ID)
  {
    portENTER_CRITICAL_ISR(&s_mux);
    ++s_badLink;
    portEXIT_CRITICAL_ISR(&s_mux);
    return;
  }

  PacketMode packetMode = PacketMode::NONE;
  if(!decodePacketMode(packet.flags, packetMode) ||
     (packetMode != PacketMode::NONE && !validChannelValues(packet)))
  {
    portENTER_CRITICAL_ISR(&s_mux);
    ++s_badValue;
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

} // namespace Espfc::Device

#endif
