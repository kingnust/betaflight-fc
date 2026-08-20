#pragma once

#include <stdint.h>

namespace Espfc::Control {

struct DroneProtoReceiverSafetyInput
{
  bool failsafeActive = false;
  bool rxLoss = false;
  bool rxFailSafe = false;
  uint32_t frameCount = 0;
  bool channelsValid = false;
};

struct DroneProtoReceiverSafetyDecision
{
  bool blockFailsafe = false;
  bool blockReceiver = false;
  bool blockNoFrame = false;

  constexpr bool ready() const
  {
    return !blockFailsafe && !blockReceiver && !blockNoFrame;
  }
};

constexpr DroneProtoReceiverSafetyDecision evaluateReceiverSafety(
  const DroneProtoReceiverSafetyInput& input)
{
  return DroneProtoReceiverSafetyDecision{
    input.failsafeActive,
    input.rxLoss || input.rxFailSafe,
    input.frameCount < 5 || !input.channelsValid,
  };
}

} // namespace Espfc::Control
