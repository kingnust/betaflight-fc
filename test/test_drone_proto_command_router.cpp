#include "Control/DroneProtoCommandRouter.hpp"

#include <array>
#include <cstdint>
#include <iostream>

using Espfc::Control::DroneProtoCommandRouter;
using Espfc::Control::DroneProtoCommandState;
using Espfc::Control::DroneProtoInputSource;
using Espfc::Control::DroneProtoTaskCommand;
using Espfc::Control::DroneProtoTaskRequest;

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if(!(condition)) { \
    std::cerr << "FAIL line " << __LINE__ << ": " #condition "\n"; \
    failures++; \
  } \
} while(false)

using Channels = std::array<uint16_t, 16>;

Channels neutralChannels()
{
  Channels channels{};
  channels.fill(1500);
  return channels;
}

void route(Channels& logical, const Channels& transport, bool directActive,
           uint32_t nowMs, DroneProtoCommandState& state)
{
  DroneProtoCommandRouter::route(logical.data(), transport.data(), transport.size(),
                                 directActive, nowMs, state);
}

void sendTrainerHeartbeat(Channels& logical, Channels& transport,
                          uint32_t startMs, DroneProtoCommandState& state)
{
  const uint16_t heartbeat[] = {1230, 1270, 1230, 1270};
  for(size_t i = 0; i < 4; i++)
  {
    transport[15] = heartbeat[i];
    logical = transport;
    route(logical, transport, false, startMs + static_cast<uint32_t>(i * 20), state);
  }
}

void testNormalMidpointDoesNotTakeOver()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  Channels logical = transport;
  transport[15] = 1500;

  route(logical, transport, false, 10, state);

  CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!state.pending.valid);
}

void testFrozenHighMarkerDoesNotTakeOver()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  Channels logical = transport;
  transport[15] = 2012; // Radio output can remain high after the trainer powers off.

  for(uint32_t nowMs: {10u, 30u, 60u, 120u, 500u})
  {
    logical = transport;
    route(logical, transport, false, nowMs, state);
    CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
    CHECK(!state.trainerHeartbeatFresh);
  }
}

void testTrainerRemapAndPreservedChannels()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  transport[5] = 2012;  // CH6 flight mode.
  transport[7] = 1750;  // CH8 servo.
  transport[10] = 1600; // Phone roll.
  transport[11] = 1400; // Phone pitch.
  transport[12] = 1300; // Phone throttle.
  transport[13] = 1700; // Phone yaw.
  transport[14] = 2000; // Phone arm.
  Channels logical = transport;

  sendTrainerHeartbeat(logical, transport, 20, state);

  CHECK(state.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(logical[0] == 1600);
  CHECK(logical[1] == 1400);
  CHECK(logical[2] == 1700);
  CHECK(logical[3] == 1300);
  CHECK(logical[4] == 2000);
  CHECK(logical[5] == 2012);
  CHECK(logical[6] == 1000);
  CHECK(logical[7] == 1750);
  for(size_t i = 8; i < logical.size(); i++) CHECK(logical[i] == 1000);
  CHECK(state.trainerTaskArmed);
  CHECK(!state.pending.valid);
}

void testTrainerTaskRequiresHeartbeatAndRearm()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  Channels logical = transport;

  sendTrainerHeartbeat(logical, transport, 20, state);
  CHECK(state.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(state.trainerTaskArmed);

  transport[15] = 1280;
  logical = transport;
  route(logical, transport, false, 100, state);
  CHECK(state.pending.valid);
  CHECK(state.pending.command == DroneProtoTaskCommand::GO_TO_PRESET_1);
  CHECK(state.pending.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(state.pending.receivedAtMs == 100);
  CHECK(state.pending.sequence == 1);

  DroneProtoTaskRequest request;
  CHECK(DroneProtoCommandRouter::consumePending(state, request));
  CHECK(request.command == DroneProtoTaskCommand::GO_TO_PRESET_1);
  CHECK(!DroneProtoCommandRouter::consumePending(state, request));

  logical = transport;
  route(logical, transport, false, 120, state);
  CHECK(!state.pending.valid);
  CHECK(state.requestSequence == 1);

  transport[15] = 1230;
  logical = transport;
  route(logical, transport, false, 140, state);
  transport[15] = 1780;
  logical = transport;
  route(logical, transport, false, 160, state);
  CHECK(state.pending.valid);
  CHECK(state.pending.command == DroneProtoTaskCommand::TASK_1);
  CHECK(state.pending.sequence == 2);

  logical = transport;
  route(logical, transport, false, 500, state);
  CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!state.trainerHeartbeatFresh);
}

void testStartupExecuteHighDoesNotTrigger()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  Channels logical = transport;
  transport[8] = 1200;
  transport[9] = 2000;

  route(logical, transport, false, 100, state);

  CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(state.selected == DroneProtoTaskCommand::GO_TO_PRESET_1);
  CHECK(!state.pending.valid);

  transport[9] = 1000;
  route(logical, transport, false, 110, state);
  transport[9] = 2000;
  route(logical, transport, false, 120, state);
  CHECK(state.pending.valid);
  CHECK(state.pending.command == DroneProtoTaskCommand::GO_TO_PRESET_1);
  CHECK(state.pending.source == DroneProtoInputSource::RADIOMASTER);
}

void testDirectSourceWinsAndUsesTaskEdge()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  Channels logical = transport;
  transport[15] = 1250;
  transport[8] = 1600;
  transport[9] = 1000;

  route(logical, transport, true, 130, state);
  CHECK(state.source == DroneProtoInputSource::WIFI_DIRECT);
  CHECK(state.selected == DroneProtoTaskCommand::RETURN_HOME);
  CHECK(!state.pending.valid);

  transport[9] = 2000;
  route(logical, transport, true, 140, state);
  CHECK(state.pending.valid);
  CHECK(state.pending.command == DroneProtoTaskCommand::RETURN_HOME);
  CHECK(state.pending.source == DroneProtoInputSource::WIFI_DIRECT);
}

}

int main()
{
  testNormalMidpointDoesNotTakeOver();
  testFrozenHighMarkerDoesNotTakeOver();
  testTrainerRemapAndPreservedChannels();
  testTrainerTaskRequiresHeartbeatAndRearm();
  testStartupExecuteHighDoesNotTrigger();
  testDirectSourceWinsAndUsesTaskEdge();

  if(failures != 0)
  {
    std::cerr << failures << " router test(s) failed\n";
    return 1;
  }

  std::cout << "All DroneProtoCommandRouter tests passed\n";
  return 0;
}
