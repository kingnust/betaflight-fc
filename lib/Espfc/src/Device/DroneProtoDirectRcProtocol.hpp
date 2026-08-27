#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Espfc::Device::DirectRcProtocol {

constexpr uint32_t MAGIC = 0x31524344UL;  // "DRC1" little-endian
constexpr uint8_t VERSION = 2;
constexpr size_t CHANNEL_COUNT = 16;
constexpr uint16_t CHANNEL_MIN_US = 988;
constexpr uint16_t CHANNEL_MID_US = 1500;
constexpr uint16_t CHANNEL_MAX_US = 2012;

// Recovered from the RadioMaster bridge's ESP32-S3 upload history. ESP-NOW
// currently transmits on WIFI_IF_AP, whose universal MAC is normally base+1.
// Accepting the base address too keeps the policy safe if the interface is
// deliberately changed to WIFI_IF_STA later.
constexpr uint8_t RADIO_BRIDGE_BASE_MAC[6] = {0x68, 0xee, 0x8f, 0xdd, 0x1f, 0x98};
constexpr uint8_t RADIO_BRIDGE_AP_MAC[6] = {0x68, 0xee, 0x8f, 0xdd, 0x1f, 0x99};

enum class Mode : uint8_t
{
  NONE,
  TRAINER_SIDEBAND,
  DIRECT,
};

enum class DecodeResult : uint8_t
{
  ACCEPTED,
  BAD_SIZE,
  BAD_CRC_OR_HEADER,
  BAD_LINK,
  BAD_VALUE,
};

#pragma pack(push, 1)
struct Packet
{
  uint32_t magic;
  uint32_t linkId;
  uint8_t version;
  uint8_t channelCount;
  uint8_t flags;
  uint8_t reserved;
  uint16_t sequence;
  uint32_t timeMs;
  uint16_t channels[CHANNEL_COUNT];
  uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(Packet) == 52, "Direct RC packet layout changed");

inline uint16_t crc16Ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xffff;
  while(len--)
  {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for(uint8_t i = 0; i < 8; ++i)
    {
      crc = (crc & 0x8000)
        ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
        : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline bool trustedSourceMac(const uint8_t *mac)
{
  if(mac == nullptr) return false;
  return std::memcmp(mac, RADIO_BRIDGE_BASE_MAC, sizeof(RADIO_BRIDGE_BASE_MAC)) == 0 ||
    std::memcmp(mac, RADIO_BRIDGE_AP_MAC, sizeof(RADIO_BRIDGE_AP_MAC)) == 0;
}

inline bool validChannelValues(const Packet& packet)
{
  for(size_t i = 0; i < CHANNEL_COUNT; ++i)
  {
    if(packet.channels[i] < CHANNEL_MIN_US || packet.channels[i] > CHANNEL_MAX_US)
    {
      return false;
    }
  }
  return true;
}

inline bool decodeMode(uint8_t flags, Mode& mode)
{
  switch(flags)
  {
    case 0x00: mode = Mode::NONE; return true;
    case 0x03: mode = Mode::DIRECT; return true;
    case 0x04: mode = Mode::TRAINER_SIDEBAND; return true;
    default: return false;
  }
}

inline DecodeResult decode(const uint8_t *data, size_t len, uint32_t expectedLinkId,
                           Packet& packet, Mode& mode)
{
  if(data == nullptr || len != sizeof(Packet)) return DecodeResult::BAD_SIZE;

  std::memcpy(&packet, data, sizeof(packet));
  const uint16_t crc = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet),
                                  sizeof(packet) - sizeof(packet.crc));
  if(crc != packet.crc || packet.magic != MAGIC || packet.version != VERSION ||
     packet.channelCount != CHANNEL_COUNT)
  {
    return DecodeResult::BAD_CRC_OR_HEADER;
  }
  if(packet.linkId != expectedLinkId) return DecodeResult::BAD_LINK;
  if(!decodeMode(packet.flags, mode) ||
     (mode != Mode::NONE && !validChannelValues(packet)))
  {
    return DecodeResult::BAD_VALUE;
  }
  return DecodeResult::ACCEPTED;
}

} // namespace Espfc::Device::DirectRcProtocol
