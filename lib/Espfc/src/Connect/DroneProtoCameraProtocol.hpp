#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Espfc::Connect::DroneProtoCameraProtocol {

constexpr uint16_t MSP2_CAMERA_QR = 0x3001;
constexpr uint8_t VERSION_V1 = 1;
constexpr uint8_t VERSION_V2 = 2;
constexpr uint8_t MESSAGE_QR = 1;
constexpr size_t MAX_TEXT_LENGTH = 96;
constexpr size_t V1_HEADER_SIZE = 5;
constexpr size_t V2_HEADER_SIZE = 16;

constexpr uint8_t FLAG_GEOMETRY_VALID = 1 << 0;
constexpr uint8_t FLAG_FULL_RESOLUTION = 1 << 1;
constexpr uint8_t FLAG_MIRRORED = 1 << 2;
constexpr uint8_t FLAG_ZBAR_FALLBACK = 1 << 3;

enum class DecodeResult : uint8_t
{
  ACCEPTED,
  MALFORMED,
  UNSUPPORTED,
};

enum AckStatus : uint8_t
{
  ACK_ACCEPTED = 0,
  ACK_DUPLICATE = 1,
  ACK_MALFORMED = 2,
  ACK_UNSUPPORTED = 3,
};

struct QrMessage
{
  uint8_t version = VERSION_V2;
  uint8_t type = 0;
  uint16_t sequence = 0;
  uint8_t length = 0;
  uint8_t flags = 0;
  uint16_t centerXPermille = 0;
  uint16_t centerYPermille = 0;
  uint16_t sidePermille = 0;
  uint16_t areaPermille = 0;
  int16_t rotationCdeg = 0;
  char payload[MAX_TEXT_LENGTH + 1] = {};
};

inline uint16_t readU16(const uint8_t *data)
{
  return static_cast<uint16_t>(data[0]) |
    (static_cast<uint16_t>(data[1]) << 8);
}

inline DecodeResult decodeQrPayload(const uint8_t *data, size_t size, QrMessage& message)
{
  message = {};
  if(data == nullptr || size < V1_HEADER_SIZE) return DecodeResult::MALFORMED;

  message.version = data[0];
  message.type = data[1];
  message.sequence = readU16(&data[2]);
  message.length = data[4];
  if((message.version != VERSION_V1 && message.version != VERSION_V2) ||
     message.type != MESSAGE_QR)
  {
    return DecodeResult::UNSUPPORTED;
  }

  size_t headerSize = V1_HEADER_SIZE;
  if(message.version == VERSION_V2)
  {
    if(size < V2_HEADER_SIZE) return DecodeResult::MALFORMED;
    headerSize = V2_HEADER_SIZE;
    message.flags = data[5];
    message.centerXPermille = readU16(&data[6]);
    message.centerYPermille = readU16(&data[8]);
    message.sidePermille = readU16(&data[10]);
    message.areaPermille = readU16(&data[12]);
    message.rotationCdeg = static_cast<int16_t>(readU16(&data[14]));
  }

  if(message.length == 0 || message.length > MAX_TEXT_LENGTH ||
     size != headerSize + message.length)
  {
    return DecodeResult::MALFORMED;
  }

  for(size_t i = 0; i < message.length; ++i)
  {
    const uint8_t value = data[headerSize + i];
    if(value < 32 || value > 126) return DecodeResult::MALFORMED;
    message.payload[i] = static_cast<char>(value);
  }
  message.payload[message.length] = '\0';
  return DecodeResult::ACCEPTED;
}

} // namespace Espfc::Connect::DroneProtoCameraProtocol
