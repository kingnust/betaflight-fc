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

using RadioChannels = std::array<uint16_t, 32>;
using PhoneChannels = std::array<uint16_t, 16>;

RadioChannels neutralRadio()
{
  RadioChannels channels{};
  channels.fill(1500);
  channels[2] = 988;
  channels[4] = 988;
  channels[10] = 988;
  return channels;
}

PhoneChannels activePhone()
{
  PhoneChannels channels{};
  channels.fill(1500);
  channels[2] = 988;
  channels[4] = 988;
  channels[5] = 1250;
  channels[12] = 2012;
  channels[14] = 988;
  channels[15] = 1230;
  return channels;
}

struct Fixture
{
  DroneProtoCommandState state;
  RadioChannels radio = neutralRadio();
  RadioChannels logical = radio;
  PhoneChannels phone = activePhone();
  uint32_t nowMs = 0;

  void tick(uint32_t advanceMs = 20, bool phoneFresh = true,
            bool toggleHeartbeat = true, bool directActive = false)
  {
    nowMs += advanceMs;
    if(phoneFresh && toggleHeartbeat)
      phone[15] = phone[15] == 1230 ? 1270 : 1230;
    logical = radio;
    DroneProtoCommandRouter::route(logical.data(), radio.data(), radio.size(),
      directActive, phone.data(), phone.size(), phoneFresh, nowMs, state);
  }

  void runFor(uint32_t durationMs, bool phoneFresh = true,
              bool toggleHeartbeat = true)
  {
    for(uint32_t elapsed = 0; elapsed < durationMs; elapsed += 20)
      tick(20, phoneFresh, toggleHeartbeat);
  }

  void qualifyPhone()
  {
    tick(1);
    runFor(220);
    CHECK(state.trainerLinkQualified);
    CHECK(state.trainerArmLowStable);
  }

  void engageTrainer()
  {
    qualifyPhone();
    radio[10] = 2012;
    runFor(120);
    CHECK(state.source == DroneProtoInputSource::TRAINER_PHONE);
    CHECK(state.trainerTakeoverLatched);
  }
};

void testOneMillisecondArmAndTrainerGlitchIgnored()
{
  Fixture f;
  f.qualifyPhone();

  f.radio[4] = 2012;
  f.radio[10] = 2012;
  f.tick(1);
  CHECK(f.logical[4] == 1000);
  CHECK(!f.state.trainerSafetyEnabled);
  CHECK(!f.state.trainerTakeoverPending);

  f.radio[4] = 988;
  f.radio[10] = 988;
  f.tick(1);
  f.runFor(140);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!f.state.trainerSafetyEnabled);
  CHECK(!f.state.radioArmDebounced);
  CHECK(!f.state.trainerTakeoverLatched);
}

void testPendingRequestSurvivesPhoneStartupOrdering()
{
  Fixture f;
  f.tick(1, false);
  f.radio[10] = 2012;
  f.runFor(120, false);
  CHECK(f.state.trainerSafetyEnabled);
  CHECK(f.state.trainerTakeoverPending);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);

  f.runFor(240, true);
  CHECK(f.state.trainerLinkQualified);
  CHECK(f.state.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(f.state.trainerTakeoverLatched);
  CHECK(!f.state.trainerTakeoverPending);
}

void testTrainerMapsCompletePhoneFrameAndOwnsArmAfterEntry()
{
  Fixture f;
  f.phone[0] = 1111;
  f.phone[1] = 1222;
  f.phone[2] = 1333;
  f.phone[3] = 1444;
  f.phone[5] = 1400;
  f.phone[6] = 1660;
  f.phone[7] = 2012;
  f.phone[8] = 2012;
  f.phone[9] = 988;
  f.phone[10] = 2012;
  f.phone[11] = 988;
  f.phone[13] = 2012;
  f.engageTrainer();

  CHECK(f.logical[0] == 1111);
  CHECK(f.logical[1] == 1222);
  CHECK(f.logical[2] == 1444);
  CHECK(f.logical[3] == 1333);
  CHECK(f.logical[4] == 988);
  CHECK(f.logical[5] == 2012);
  CHECK(f.logical[6] == 2012);
  CHECK(f.logical[8] == 1400);
  CHECK(f.logical[9] == 1660);
  CHECK(f.logical[10] == 2012);
  CHECK(f.logical[11] == 988);
  CHECK(f.logical[12] == 2012);
  CHECK(f.logical[13] == 988);
  CHECK(f.logical[14] == 2012);
  CHECK(f.logical[15] == 2012);

  f.radio[4] = 2012;
  f.runFor(120);
  CHECK(f.state.source == DroneProtoInputSource::TRAINER_PHONE);
  CHECK(f.logical[4] == 988);

  f.phone[4] = 2012;
  f.tick();
  CHECK(f.logical[4] == 2012);
}

void testArmedRadioBlocksTakeoverWithoutDisarming()
{
  Fixture f;
  f.qualifyPhone();
  f.radio[4] = 2012;
  f.runFor(120);
  CHECK(f.state.radioArmDebounced);
  CHECK(f.logical[4] == 2012);

  f.radio[10] = 2012;
  f.runFor(120);
  CHECK(f.state.trainerTakeoverBlockedArmed);
  CHECK(!f.state.trainerTakeoverLatched);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(f.logical[4] == 2012);

  f.radio[4] = 988;
  f.runFor(140);
  CHECK(!f.state.radioArmDebounced);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!f.state.trainerTakeoverLatched);

  f.radio[10] = 988;
  f.runFor(120);
  f.radio[10] = 2012;
  f.runFor(120);
  CHECK(f.state.source == DroneProtoInputSource::TRAINER_PHONE);
}

void testSimultaneousSustainedArmAndTrainerChoosesRadioArm()
{
  Fixture f;
  f.qualifyPhone();

  f.radio[4] = 2012;
  f.radio[10] = 2012;
  f.runFor(120);
  CHECK(f.state.radioArmDebounced);
  CHECK(f.state.trainerSafetyEnabled);
  CHECK(f.state.trainerTakeoverBlockedArmed);
  CHECK(!f.state.trainerTakeoverLatched);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(f.logical[4] == 2012);
}

void testPhoneArmHighRequiresFreshTrainerCycle()
{
  Fixture f;
  f.phone[4] = 2012;
  f.tick(1);
  f.runFor(220);
  CHECK(f.state.trainerLinkQualified);
  CHECK(!f.state.trainerArmLowStable);

  f.radio[10] = 2012;
  f.runFor(120);
  CHECK(f.state.trainerTakeoverBlockedTrainerArmed);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);

  f.phone[4] = 988;
  f.runFor(220);
  CHECK(f.state.trainerArmLowStable);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);

  f.radio[10] = 988;
  f.runFor(120);
  f.radio[10] = 2012;
  f.runFor(120);
  CHECK(f.state.source == DroneProtoInputSource::TRAINER_PHONE);
}

void testTrainerIgnoresShortDropoutAndExitInterlocksRadioArm()
{
  Fixture f;
  f.radio[0] = 1750;
  f.phone[0] = 1100;
  f.engageTrainer();

  f.radio[4] = 2012;
  f.runFor(120);
  CHECK(f.logical[4] == 988);

  f.radio[10] = 988;
  f.tick(1);
  f.radio[10] = 2012;
  f.tick(1);
  f.runFor(120);
  CHECK(f.state.source == DroneProtoInputSource::TRAINER_PHONE);

  f.runFor(200, true, false);
  CHECK(f.state.source == DroneProtoInputSource::TRAINER_PHONE);
  f.tick(20, true, true);
  CHECK(f.state.source == DroneProtoInputSource::TRAINER_PHONE);

  f.radio[10] = 988;
  f.runFor(120);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!f.state.trainerTakeoverLatched);
  CHECK(f.logical[0] == 1750);
  CHECK(f.logical[4] == 1000);
  CHECK(f.state.radioArmReleaseRequired);

  f.radio[4] = 988;
  f.tick(1);
  f.radio[4] = 2012;
  f.tick(1);
  CHECK(f.state.radioArmReleaseRequired);
  CHECK(f.logical[4] == 1000);

  f.radio[4] = 988;
  f.runFor(120);
  CHECK(!f.state.radioArmReleaseRequired);
}

void testQualifiedLinkLossRestoresRadioAndRequiresNewSwitchEdge()
{
  Fixture f;
  f.engageTrainer();
  f.radio[4] = 2012;
  f.runFor(120);

  f.tick(20, false, false);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!f.state.trainerTakeoverLatched);
  CHECK(f.state.radioArmReleaseRequired);
  CHECK(f.logical[4] == 1000);

  f.radio[4] = 988;
  f.runFor(140);
  f.runFor(220, true);
  CHECK(f.state.source == DroneProtoInputSource::RADIOMASTER);
  CHECK(!f.state.trainerTakeoverLatched);
}

void testTrainerTaskUsesSelectorAndRunEdge()
{
  Fixture f;
  f.phone[5] = 1300;
  f.phone[14] = 988;
  f.engageTrainer();
  CHECK(f.state.selected == DroneProtoTaskCommand::GO_TO_PRESET_1);
  CHECK(!f.state.pending.valid);

  f.phone[14] = 2012;
  f.tick();
  CHECK(f.state.pending.valid);
  CHECK(f.state.pending.command == DroneProtoTaskCommand::GO_TO_PRESET_1);
  CHECK(f.state.pending.source == DroneProtoInputSource::TRAINER_PHONE);

  DroneProtoTaskRequest request;
  CHECK(DroneProtoCommandRouter::consumePending(f.state, request));
  CHECK(!DroneProtoCommandRouter::consumePending(f.state, request));
}

void testNormalRadioTaskAndDirectPriority()
{
  Fixture f;
  f.radio[8] = 1600;
  f.radio[7] = 988;
  f.tick();
  f.radio[7] = 2012;
  f.tick();
  CHECK(f.state.pending.valid);
  CHECK(f.state.pending.command == DroneProtoTaskCommand::RETURN_HOME);
  CHECK(f.state.pending.source == DroneProtoInputSource::RADIOMASTER);

  DroneProtoCommandRouter::reset(f.state);
  f.radio[10] = 2012;
  f.tick(20, true, true, true);
  CHECK(f.state.source == DroneProtoInputSource::WIFI_DIRECT);
  CHECK(!f.state.trainerTakeoverLatched);
}

} // namespace

int main()
{
  testOneMillisecondArmAndTrainerGlitchIgnored();
  testPendingRequestSurvivesPhoneStartupOrdering();
  testTrainerMapsCompletePhoneFrameAndOwnsArmAfterEntry();
  testArmedRadioBlocksTakeoverWithoutDisarming();
  testSimultaneousSustainedArmAndTrainerChoosesRadioArm();
  testPhoneArmHighRequiresFreshTrainerCycle();
  testTrainerIgnoresShortDropoutAndExitInterlocksRadioArm();
  testQualifiedLinkLossRestoresRadioAndRequiresNewSwitchEdge();
  testTrainerTaskUsesSelectorAndRunEdge();
  testNormalRadioTaskAndDirectPriority();

  if(failures != 0)
  {
    std::cerr << failures << " router test(s) failed\n";
    return 1;
  }

  std::cout << "All DroneProtoCommandRouter tests passed\n";
  return 0;
}
