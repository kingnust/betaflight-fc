#pragma once

#include <cstdint>

namespace Espfc::Device::Wk2132Protocol {

constexpr uint8_t CHANNEL_CAMERA = 0;
constexpr uint8_t CHANNEL_UWB = 1;
constexpr uint8_t CHANNEL_COUNT = 2;

constexpr uint8_t i2cAddress(bool ia1High, bool ia0High, uint8_t channel, bool fifo)
{
  return static_cast<uint8_t>(
    0x10u |
    (ia1High ? 0x40u : 0u) |
    (ia0High ? 0x20u : 0u) |
    ((channel & 0x01u) << 1u) |
    (fifo ? 0x01u : 0u));
}

struct BaudConfig
{
  bool valid = false;
  uint16_t divisorRegister = 0;
  uint8_t prescaler = 0;
  uint32_t actualBaud = 0;
  uint32_t errorPpm = 0;
};

constexpr BaudConfig calculateBaud(uint32_t oscillatorHz, uint32_t requestedBaud)
{
  BaudConfig result;
  if(oscillatorHz == 0 || requestedBaud == 0)
  {
    return result;
  }

  // The chip divides its oscillator by Reg * 16. BAUD stores the integer
  // portion of Reg minus one, while PRES stores its fractional sixteenths.
  const uint64_t regSixteenths =
    (static_cast<uint64_t>(oscillatorHz) + (requestedBaud / 2u)) / requestedBaud;
  constexpr uint64_t kMinRegSixteenths = 16u;
  constexpr uint64_t kMaxRegSixteenths = (65536ull * 16ull) + 15ull;
  if(regSixteenths < kMinRegSixteenths || regSixteenths > kMaxRegSixteenths)
  {
    return result;
  }

  const uint64_t integerPart = regSixteenths / 16u;
  if(integerPart == 0 || integerPart > 65536u)
  {
    return result;
  }

  result.divisorRegister = static_cast<uint16_t>(integerPart - 1u);
  result.prescaler = static_cast<uint8_t>(regSixteenths % 16u);
  result.actualBaud = static_cast<uint32_t>(
    static_cast<uint64_t>(oscillatorHz) / regSixteenths);

  const uint32_t absoluteError = result.actualBaud > requestedBaud
    ? result.actualBaud - requestedBaud
    : requestedBaud - result.actualBaud;
  result.errorPpm = static_cast<uint32_t>(
    (static_cast<uint64_t>(absoluteError) * 1000000ull) / requestedBaud);
  result.valid = result.errorPpm <= 30000u;
  return result;
}

} // namespace Espfc::Device::Wk2132Protocol
