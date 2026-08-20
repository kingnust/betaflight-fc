#include "Control/DroneProtoSafetyPolicy.hpp"

#include <iostream>

using Espfc::Control::DroneProtoReceiverSafetyInput;
using Espfc::Control::evaluateReceiverSafety;

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if(!(condition)) { \
    std::cerr << "FAIL line " << __LINE__ << ": " #condition "\n"; \
    failures++; \
  } \
} while(false)

void testHealthyReceiverIsReady()
{
  DroneProtoReceiverSafetyInput input;
  input.frameCount = 5;
  input.channelsValid = true;
  const auto decision = evaluateReceiverSafety(input);
  CHECK(decision.ready());
}

void testEveryReceiverFailureBlocksArming()
{
  DroneProtoReceiverSafetyInput input;
  input.frameCount = 10;
  input.channelsValid = true;

  input.failsafeActive = true;
  CHECK(evaluateReceiverSafety(input).blockFailsafe);
  input.failsafeActive = false;

  input.rxLoss = true;
  CHECK(evaluateReceiverSafety(input).blockReceiver);
  input.rxLoss = false;

  input.rxFailSafe = true;
  CHECK(evaluateReceiverSafety(input).blockReceiver);
  input.rxFailSafe = false;

  input.frameCount = 4;
  CHECK(evaluateReceiverSafety(input).blockNoFrame);
  input.frameCount = 10;

  input.channelsValid = false;
  CHECK(evaluateReceiverSafety(input).blockNoFrame);
}

void testReceiverCanRecoverOnlyAfterAllFaultsClear()
{
  DroneProtoReceiverSafetyInput input;
  input.failsafeActive = true;
  input.rxLoss = true;
  input.rxFailSafe = true;
  input.frameCount = 2;
  input.channelsValid = false;
  CHECK(!evaluateReceiverSafety(input).ready());

  input.failsafeActive = false;
  input.rxLoss = false;
  input.rxFailSafe = false;
  input.frameCount = 5;
  input.channelsValid = true;
  CHECK(evaluateReceiverSafety(input).ready());
}

} // namespace

int main()
{
  testHealthyReceiverIsReady();
  testEveryReceiverFailureBlocksArming();
  testReceiverCanRecoverOnlyAfterAllFaultsClear();

  if(failures != 0)
  {
    std::cerr << failures << " safety test(s) failed\n";
    return 1;
  }
  std::cout << "All DroneProto receiver safety tests passed\n";
  return 0;
}
