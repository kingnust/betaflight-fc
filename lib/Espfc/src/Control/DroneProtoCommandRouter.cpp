#include "Control/DroneProtoCommandRouter.hpp"

namespace Espfc {

namespace Control {

namespace {

constexpr size_t RADIO_TASK_SELECT_INDEX = 8;   // Physical CH9.
constexpr size_t RADIO_TASK_EXECUTE_INDEX = 7;  // Physical CH8.
constexpr size_t RADIO_TRAINER_SAFETY_INDEX = 10; // RadioMaster CH11.
constexpr size_t TRAINER_ROLL_INDEX = 0;
constexpr size_t TRAINER_PITCH_INDEX = 1;
constexpr size_t TRAINER_THROTTLE_INDEX = 2;
constexpr size_t TRAINER_YAW_INDEX = 3;
constexpr size_t TRAINER_ARM_INDEX = 4;
constexpr size_t TRAINER_TASK_INDEX = 5;
constexpr size_t TRAINER_SERVO_INDEX = 6;
constexpr size_t TRAINER_MODE_INDEX = 7;
constexpr size_t TRAINER_BEEP_INDEX = 8;
constexpr size_t TRAINER_AUX6_INDEX = 9;
constexpr size_t TRAINER_AUX7_INDEX = 10;
constexpr size_t TRAINER_AUX8_INDEX = 11;
constexpr size_t TRAINER_TAKEOVER_INDEX = 12;
constexpr size_t TRAINER_AIR_INDEX = 13;
constexpr size_t TRAINER_EXECUTE_INDEX = 14;
constexpr size_t TRAINER_HEARTBEAT_INDEX = 15;
constexpr uint16_t EXECUTE_HIGH_US = 1700;
constexpr uint16_t ARM_HIGH_US = 1700;
constexpr uint16_t TRAINER_HEARTBEAT_MIN_US = 1200;
constexpr uint16_t TRAINER_HEARTBEAT_MAX_US = 1300;
constexpr uint16_t TRAINER_HEARTBEAT_STEP_US = 20;
constexpr uint8_t TRAINER_HEARTBEAT_REQUIRED_TRANSITIONS = 3;
constexpr uint32_t TRAINER_HEARTBEAT_TIMEOUT_MS = 250;
constexpr uint32_t TRAINER_SWITCH_DEBOUNCE_MS = 100;
constexpr uint32_t RADIO_ARM_DEBOUNCE_MS = 100;
constexpr uint32_t TRAINER_ARM_LOW_MS = 100;
constexpr uint32_t TRAINER_PENDING_TIMEOUT_MS = 2500;

bool updateDebouncedSwitch(bool raw,
                           uint32_t nowMs,
                           uint32_t debounceMs,
                           bool& initialized,
                           bool& candidate,
                           uint32_t& candidateSinceMs,
                           bool& stable)
{
  if(!initialized)
  {
    initialized = true;
    candidate = raw;
    candidateSinceMs = nowMs;
    stable = raw;
    return false;
  }

  if(raw != candidate)
  {
    candidate = raw;
    candidateSinceMs = nowMs;
    return false;
  }

  if(candidate != stable && nowMs - candidateSinceMs >= debounceMs)
  {
    stable = candidate;
    return true;
  }
  return false;
}

DroneProtoTaskCommand decodeTaskSelector(uint16_t value)
{
  if(value < 1100) return DroneProtoTaskCommand::NONE;
  if(value < 1300) return DroneProtoTaskCommand::GO_TO_PRESET_1;
  if(value < 1500) return DroneProtoTaskCommand::GO_TO_PRESET_2;
  if(value < 1700) return DroneProtoTaskCommand::RETURN_HOME;
  if(value < 1900) return DroneProtoTaskCommand::TASK_1;
  return DroneProtoTaskCommand::TASK_2;
}

DroneProtoTaskCommand decodeTrainerTask(uint16_t value)
{
  if(value < 1275) return DroneProtoTaskCommand::NONE;
  if(value < 1350) return DroneProtoTaskCommand::GO_TO_PRESET_1;
  if(value < 1450) return DroneProtoTaskCommand::GO_TO_PRESET_2;
  if(value < 1700) return DroneProtoTaskCommand::RETURN_HOME;
  if(value < 1900) return DroneProtoTaskCommand::TASK_1;
  return DroneProtoTaskCommand::TASK_2;
}

void queueRequest(DroneProtoCommandState& state, DroneProtoTaskCommand command, uint32_t nowMs)
{
  if(command == DroneProtoTaskCommand::NONE) return;
  if(state.pending.valid) state.overwrittenRequests++;

  state.requestSequence++;
  state.pending.valid = true;
  state.pending.command = command;
  state.pending.source = state.source;
  state.pending.sequence = state.requestSequence;
  state.pending.receivedAtMs = nowMs;
  for(size_t i = 0; i < DRONE_PROTO_FUNCTION_CHANNELS; i++)
  {
    state.pending.functionUs[i] = state.functionUs[i];
  }
}

}

void DroneProtoCommandRouter::reset(DroneProtoCommandState& state)
{
  state = DroneProtoCommandState{};
}

void DroneProtoCommandRouter::route(uint16_t *logicalChannels,
                                    const uint16_t *transportChannels,
                                    size_t channelCount,
                                    bool directActive,
                                    const uint16_t *trainerSidebandChannels,
                                    size_t trainerSidebandChannelCount,
                                    bool trainerSidebandFresh,
                                    uint32_t nowMs,
                                    DroneProtoCommandState& state)
{
  if(logicalChannels == nullptr || transportChannels == nullptr) return;
  const bool trainerFrameComplete = trainerSidebandChannels != nullptr &&
    trainerSidebandChannelCount >= DRONE_PROTO_FUNCTION_CHANNELS;
  const bool trainerPacketFresh = trainerFrameComplete && trainerSidebandFresh;

  for(size_t i = 0; i < DRONE_PROTO_FUNCTION_CHANNELS; i++)
  {
    state.functionUs[i] = trainerPacketFresh ? trainerSidebandChannels[i] : 1500;
  }

  state.trainerSafetyUs = RADIO_TRAINER_SAFETY_INDEX < channelCount
    ? logicalChannels[RADIO_TRAINER_SAFETY_INDEX]
    : 1000;
  state.trainerSafetyRaw = state.trainerSafetyUs >= EXECUTE_HIGH_US;
  const bool trainerSafetyChanged = updateDebouncedSwitch(
    state.trainerSafetyRaw, nowMs, TRAINER_SWITCH_DEBOUNCE_MS,
    state.trainerSafetyInitialized, state.trainerSafetyCandidate,
    state.trainerSafetyCandidateSinceMs, state.trainerSafetyEnabled);
  state.trainerSafetyRising = trainerSafetyChanged && state.trainerSafetyEnabled;

  state.radioArmUs = channelCount > 4 ? logicalChannels[4] : 1000;
  state.radioArmRaw = state.radioArmUs >= ARM_HIGH_US;
  updateDebouncedSwitch(state.radioArmRaw, nowMs, RADIO_ARM_DEBOUNCE_MS,
    state.radioArmInitialized, state.radioArmCandidate,
    state.radioArmCandidateSinceMs, state.radioArmDebounced);

  state.trainerTakeoverUs = trainerPacketFresh
    ? trainerSidebandChannels[TRAINER_TAKEOVER_INDEX]
    : 1000;
  state.trainerArmUs = trainerPacketFresh
    ? trainerSidebandChannels[TRAINER_ARM_INDEX]
    : 1000;
  state.trainerHeartbeatUs = trainerPacketFresh
    ? trainerSidebandChannels[TRAINER_HEARTBEAT_INDEX]
    : 1000;

  state.trainerTakeoverRequested = trainerPacketFresh &&
    state.trainerTakeoverUs >= EXECUTE_HIGH_US;
  const bool trainerArmed = trainerPacketFresh && state.trainerArmUs >= ARM_HIGH_US;
  const bool heartbeatInRange = trainerPacketFresh &&
    state.trainerHeartbeatUs >= TRAINER_HEARTBEAT_MIN_US &&
    state.trainerHeartbeatUs <= TRAINER_HEARTBEAT_MAX_US;

  if(!heartbeatInRange || !state.trainerTakeoverRequested)
  {
    state.trainerHeartbeatReferenceUs = 0;
    state.trainerHeartbeatLastTransitionMs = 0;
    state.trainerHeartbeatTransitions = 0;
    state.trainerHeartbeatFresh = false;
  }
  else if(state.trainerHeartbeatReferenceUs == 0)
  {
    state.trainerHeartbeatReferenceUs = state.trainerHeartbeatUs;
    state.trainerHeartbeatFresh = false;
  }
  else
  {
    const uint16_t heartbeatDelta = state.trainerHeartbeatUs > state.trainerHeartbeatReferenceUs
      ? state.trainerHeartbeatUs - state.trainerHeartbeatReferenceUs
      : state.trainerHeartbeatReferenceUs - state.trainerHeartbeatUs;
    if(heartbeatDelta >= TRAINER_HEARTBEAT_STEP_US)
    {
      state.trainerHeartbeatReferenceUs = state.trainerHeartbeatUs;
      state.trainerHeartbeatLastTransitionMs = nowMs;
      if(state.trainerHeartbeatTransitions < 255) state.trainerHeartbeatTransitions++;
    }
    state.trainerHeartbeatFresh =
      state.trainerHeartbeatTransitions >= TRAINER_HEARTBEAT_REQUIRED_TRANSITIONS &&
      state.trainerHeartbeatLastTransitionMs != 0 &&
      nowMs - state.trainerHeartbeatLastTransitionMs <= TRAINER_HEARTBEAT_TIMEOUT_MS;
  }

  state.trainerLinkQualified = trainerPacketFresh &&
    state.trainerTakeoverRequested && state.trainerHeartbeatFresh;
  if(state.trainerLinkQualified && !trainerArmed)
  {
    if(state.trainerArmLowSinceMs == 0) state.trainerArmLowSinceMs = nowMs;
    state.trainerArmLowStable = nowMs - state.trainerArmLowSinceMs >= TRAINER_ARM_LOW_MS;
  }
  else
  {
    state.trainerArmLowSinceMs = 0;
    state.trainerArmLowStable = false;
  }

  const bool trainerWasLatched = state.trainerTakeoverLatched;

  if(directActive || !state.trainerSafetyEnabled || !state.trainerLinkQualified)
  {
    state.trainerTakeoverLatched = false;
  }

  if(state.trainerSafetyRising && !directActive)
  {
    state.trainerTakeoverTimedOut = false;
    state.trainerTakeoverBlockedArmed = state.radioArmDebounced;
    state.trainerTakeoverBlockedTrainerArmed = false;
    state.trainerTakeoverPending = !state.radioArmDebounced;
    state.trainerTakeoverPendingSinceMs = nowMs;
  }

  if(state.trainerTakeoverPending)
  {
    if(directActive || !state.trainerSafetyEnabled)
    {
      state.trainerTakeoverPending = false;
    }
    else if(state.radioArmDebounced)
    {
      state.trainerTakeoverPending = false;
      state.trainerTakeoverBlockedArmed = true;
    }
    else if(state.trainerLinkQualified && trainerArmed)
    {
      state.trainerTakeoverPending = false;
      state.trainerTakeoverBlockedTrainerArmed = true;
    }
    else if(state.trainerLinkQualified && state.trainerArmLowStable && !state.radioArmRaw)
    {
      state.trainerTakeoverPending = false;
      state.trainerTakeoverLatched = true;
    }
    else if(nowMs - state.trainerTakeoverPendingSinceMs >= TRAINER_PENDING_TIMEOUT_MS)
    {
      state.trainerTakeoverPending = false;
      state.trainerTakeoverTimedOut = true;
    }
  }

  if(!state.trainerSafetyEnabled)
  {
    state.trainerTakeoverPending = false;
    state.trainerTakeoverTimedOut = false;
    state.trainerTakeoverBlockedArmed = false;
    state.trainerTakeoverBlockedTrainerArmed = false;
  }

  const bool trainerActive = !directActive && state.trainerSafetyEnabled &&
    state.trainerLinkQualified && state.trainerTakeoverLatched;
  if(trainerWasLatched && !trainerActive)
  {
    // Returning to RadioMaster control must never inherit an arm switch that
    // was moved while trainer owned the drone. Require a physical low first.
    state.radioArmReleaseRequired = true;
  }
  else if(trainerActive)
  {
    state.radioArmReleaseRequired = false;
    state.trainerTakeoverPending = false;
    state.trainerTakeoverBlockedArmed = false;
    state.trainerTakeoverBlockedTrainerArmed = false;
  }

  const DroneProtoInputSource nextSource = directActive
    ? DroneProtoInputSource::WIFI_DIRECT
    : (trainerActive ? DroneProtoInputSource::TRAINER_PHONE : DroneProtoInputSource::RADIOMASTER);

  if(nextSource != state.source)
  {
    state.source = nextSource;
    state.executeInitialized = false;
    state.executeHigh = false;
    state.selected = DroneProtoTaskCommand::NONE;
  }

  if(trainerActive)
  {
    // A latched trainer frame owns all flight controls, including arm. The
    // physical CH11 permit remains outside the trainer data path as the exit.
    const auto copyTrainer = [&](size_t logicalIndex, size_t sidebandIndex)
    {
      if(logicalIndex < channelCount && sidebandIndex < trainerSidebandChannelCount)
        logicalChannels[logicalIndex] = trainerSidebandChannels[sidebandIndex];
    };
    copyTrainer(0, TRAINER_ROLL_INDEX);
    copyTrainer(1, TRAINER_PITCH_INDEX);
    copyTrainer(2, TRAINER_YAW_INDEX);
    copyTrainer(3, TRAINER_THROTTLE_INDEX);
    copyTrainer(4, TRAINER_ARM_INDEX);
    copyTrainer(5, TRAINER_MODE_INDEX);
    copyTrainer(6, TRAINER_AIR_INDEX);
    copyTrainer(7, TRAINER_EXECUTE_INDEX);
    copyTrainer(8, TRAINER_TASK_INDEX);
    copyTrainer(9, TRAINER_SERVO_INDEX);
    copyTrainer(11, TRAINER_AUX6_INDEX);
    copyTrainer(12, TRAINER_AUX7_INDEX);
    copyTrainer(13, TRAINER_AUX8_INDEX);
    copyTrainer(14, TRAINER_BEEP_INDEX);
    copyTrainer(15, TRAINER_TAKEOVER_INDEX);

    state.selected = decodeTrainerTask(trainerSidebandChannels[TRAINER_TASK_INDEX]);
    const bool executeHigh = trainerSidebandChannels[TRAINER_EXECUTE_INDEX] >= EXECUTE_HIGH_US;
    if(!state.executeInitialized)
    {
      state.executeInitialized = true;
      state.executeHigh = executeHigh;
    }
    else if(executeHigh && !state.executeHigh)
    {
      queueRequest(state, state.selected, nowMs);
    }
    state.executeHigh = executeHigh;
    return;
  }

  if(nextSource == DroneProtoInputSource::RADIOMASTER &&
     channelCount > 4)
  {
    // A short high pulse must not arm the FC. A low command remains immediate,
    // while arming requires a stable physical switch for the debounce period.
    if(!state.radioArmRaw || !state.radioArmDebounced)
    {
      logicalChannels[4] = 1000;
    }
    if(state.radioArmReleaseRequired)
    {
      if(state.radioArmRaw || state.radioArmDebounced)
      {
        logicalChannels[4] = 1000;
      }
      else
      {
        state.radioArmReleaseRequired = false;
      }
    }
  }

  if(channelCount <= RADIO_TASK_EXECUTE_INDEX)
  {
    state.selected = DroneProtoTaskCommand::NONE;
    state.executeInitialized = false;
    state.executeHigh = false;
    return;
  }

  state.selected = decodeTaskSelector(transportChannels[RADIO_TASK_SELECT_INDEX]);
  const bool executeHigh = transportChannels[RADIO_TASK_EXECUTE_INDEX] >= EXECUTE_HIGH_US;
  if(!state.executeInitialized)
  {
    state.executeInitialized = true;
    state.executeHigh = executeHigh;
    return;
  }

  if(executeHigh && !state.executeHigh)
  {
    queueRequest(state, state.selected, nowMs);
  }
  state.executeHigh = executeHigh;
}

bool DroneProtoCommandRouter::consumePending(DroneProtoCommandState& state, DroneProtoTaskRequest& request)
{
  if(!state.pending.valid) return false;
  request = state.pending;
  state.pending.valid = false;
  return true;
}

const char * DroneProtoCommandRouter::sourceName(DroneProtoInputSource source)
{
  switch(source)
  {
    case DroneProtoInputSource::TRAINER_PHONE: return "TRAINER_PHONE";
    case DroneProtoInputSource::WIFI_DIRECT: return "WIFI_DIRECT";
    case DroneProtoInputSource::RADIOMASTER:
    default: return "RADIOMASTER";
  }
}

const char * DroneProtoCommandRouter::taskName(DroneProtoTaskCommand task)
{
  switch(task)
  {
    case DroneProtoTaskCommand::GO_TO_PRESET_1: return "GO_TO_PRESET_1";
    case DroneProtoTaskCommand::GO_TO_PRESET_2: return "GO_TO_PRESET_2";
    case DroneProtoTaskCommand::RETURN_HOME: return "RETURN_HOME";
    case DroneProtoTaskCommand::TASK_1: return "TASK_1";
    case DroneProtoTaskCommand::TASK_2: return "TASK_2";
    case DroneProtoTaskCommand::NONE:
    default: return "NONE";
  }
}

}

}
