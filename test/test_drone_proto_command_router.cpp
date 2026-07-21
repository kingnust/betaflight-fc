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
           uint32_t nowMs, DroneProtoCommandState& state,
           const Channels *trainerSideband = nullptr, bool trainerSidebandFresh = false)
{
  DroneProtoCommandRouter::route(logical.data(), transport.data(), transport.size(),
                                 directActive,
                                 trainerSideband ? trainerSideband->data() : nullptr,
                                 trainerSideband ? trainerSideband->size() : 0,
                                 trainerSidebandFresh, nowMs, state);
}

void sendTrainerHeartbeat(Channels& logical, Channels& transport,
                          uint32_t startMs, DroneProtoCommandState& state,
                          const Channels *trainerSideband = nullptr,
                          bool trainerSidebandFresh = false)
{
  const uint16_t heartbeat[] = {1230, 1270, 1230, 1270};
  for(size_t i = 0; i < 4; i++)
  {
    transport[15] = heartbeat[i];
    logical = transport;
    route(logical, transport, false, startMs + static_cast<uint32_t>(i * 20), state,
          trainerSideband, trainerSidebandFresh);
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

void testHeartbeatWithoutPhoneFrameKeepsRadioMaster()
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

  CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(state.trainerHeartbeatFresh);
  for(size_t i = 0; i < logical.size(); i++) CHECK(logical[i] == transport[i]);
  CHECK(!state.trainerTaskArmed);
  CHECK(!state.pending.valid);
}

void testTrainerFullPhoneOverride()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  transport[5] = 1400;
  transport[7] = 1450;
  transport[10] = 1600;
  transport[11] = 1400;
  transport[12] = 1300;
  transport[13] = 1700;
  transport[14] = 988;

  Channels sideband = neutralChannels();
  sideband[0] = 1111;  // Roll.
  sideband[1] = 1222;  // Pitch.
  sideband[2] = 1333;  // Throttle.
  sideband[3] = 1444;  // Yaw.
  sideband[4] = 2012;  // Arm.
  sideband[5] = 1250;  // Task selector idle.
  sideband[6] = 1660;  // Servo.
  sideband[7] = 2012;  // Flight mode.
  sideband[8] = 2012;  // Beeper.
  sideband[9] = 988;   // Aux6.
  sideband[10] = 2012; // Aux7.
  sideband[11] = 988;  // Aux8.
  sideband[12] = 2012; // Takeover/deadman.
  sideband[13] = 2012; // Air mode.
  sideband[14] = 2012; // Task run.
  sideband[15] = 2012; // Aux5.

  Channels logical = transport;
  sendTrainerHeartbeat(logical, transport, 20, state, &sideband, true);

  CHECK(state.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(logical[0] == 1111);
  CHECK(logical[1] == 1222);
  CHECK(logical[2] == 1444);
  CHECK(logical[3] == 1333);
  CHECK(logical[4] == 2012);
  CHECK(logical[5] == 2012);
  CHECK(logical[6] == 2012);
  CHECK(logical[7] == 1660);
  CHECK(logical[8] == 1250);
  CHECK(logical[9] == 2012);
  CHECK(logical[10] == 2012);
  CHECK(logical[11] == 988);
  CHECK(logical[12] == 2012);
  CHECK(logical[13] == 988);
  CHECK(logical[14] == 2012);
  CHECK(logical[15] == 2012);
  for(size_t i = 0; i < 6; i++) CHECK(state.functionUs[i] == logical[10 + i]);
}

void testTrainerSidebandRequiresHeartbeat()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  transport[5] = 1300;
  transport[15] = 1500;
  Channels sideband = neutralChannels();
  sideband[7] = 2012;
  Channels logical = transport;

  route(logical, transport, false, 100, state, &sideband, true);

  CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!state.trainerHeartbeatFresh);
  CHECK(logical[5] == 1300);
}

void testTrainerSidebandLossRestoresWholeRadioFrame()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  transport[5] = 1500;
  transport[7] = 1600;
  Channels sideband = neutralChannels();
  sideband[7] = 2012;
  sideband[8] = 2012;
  sideband[13] = 2012;
  Channels logical = transport;

  sendTrainerHeartbeat(logical, transport, 20, state, &sideband, true);
  CHECK(logical[5] == 2012);
  CHECK(logical[6] == 2012);
  CHECK(logical[14] == 2012);

  transport[15] = 1230;
  logical = transport;
  route(logical, transport, false, 100, state, nullptr, false);

  CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(logical[5] == 1500);
  CHECK(logical[7] == 1600);
  CHECK(logical[6] == 1500);
  CHECK(logical[14] == 1500);
}

void testRadioTrainerSwitchTurnsOffTakeover()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  transport[0] = 1750;
  Channels sideband = neutralChannels();
  sideband[0] = 1100;
  sideband[4] = 2012;
  Channels logical = transport;

  sendTrainerHeartbeat(logical, transport, 20, state, &sideband, true);
  CHECK(state.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(logical[0] == 1100);
  CHECK(logical[4] == 2012);

  // A RadioMaster trainer switch that stops passing the heartbeat must have
  // final authority even while fresh phone frames continue over ESP-NOW.
  logical = transport;
  route(logical, transport, false, 500, state, &sideband, true);
  CHECK(state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!state.trainerHeartbeatFresh);
  CHECK(logical[0] == 1750);
  CHECK(logical[4] == transport[4]);
}

void testTrainerTaskRequiresHeartbeatAndRearm()
{
  DroneProtoCommandState state;
  DroneProtoCommandRouter::reset(state);
  Channels transport = neutralChannels();
  Channels sideband = neutralChannels();
  sideband[5] = 1250;
  Channels logical = transport;

  sendTrainerHeartbeat(logical, transport, 20, state, &sideband, true);
  CHECK(state.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(state.trainerTaskArmed);

  transport[15] = 1280;
  logical = transport;
  route(logical, transport, false, 100, state, &sideband, true);
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
  route(logical, transport, false, 120, state, &sideband, true);
  CHECK(!state.pending.valid);
  CHECK(state.requestSequence == 1);

  transport[15] = 1230;
  logical = transport;
  route(logical, transport, false, 140, state, &sideband, true);
  transport[15] = 1780;
  logical = transport;
  route(logical, transport, false, 160, state, &sideband, true);
  CHECK(state.pending.valid);
  CHECK(state.pending.command == DroneProtoTaskCommand::TASK_1);
  CHECK(state.pending.sequence == 2);

  logical = transport;
  route(logical, transport, false, 500, state, &sideband, true);
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
  testHeartbeatWithoutPhoneFrameKeepsRadioMaster();
  testTrainerFullPhoneOverride();
  testTrainerSidebandRequiresHeartbeat();
  testTrainerSidebandLossRestoresWholeRadioFrame();
  testRadioTrainerSwitchTurnsOffTakeover();
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
