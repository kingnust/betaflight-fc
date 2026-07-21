#include "Control/DroneProtoCommandRouter.hpp"

namespace Espfc {

namespace Control {

namespace {

constexpr size_t RADIO_TASK_SELECT_INDEX = 8;   // Physical CH9.
constexpr size_t RADIO_TASK_EXECUTE_INDEX = 9;  // Physical CH10.
constexpr size_t TRAINER_ROLL_INDEX = 10;       // Physical CH11 through CH15.
constexpr size_t TRAINER_MARKER_INDEX = 15;     // Physical CH16.
constexpr size_t TRAINER_SIDEBAND_ROLL_INDEX = 0;
constexpr size_t TRAINER_SIDEBAND_PITCH_INDEX = 1;
constexpr size_t TRAINER_SIDEBAND_THROTTLE_INDEX = 2;
constexpr size_t TRAINER_SIDEBAND_YAW_INDEX = 3;
constexpr size_t TRAINER_SIDEBAND_ARM_INDEX = 4;
constexpr size_t TRAINER_SIDEBAND_TASK_INDEX = 5;
constexpr size_t TRAINER_SIDEBAND_SERVO_INDEX = 6;
constexpr size_t TRAINER_SIDEBAND_MODE_INDEX = 7;
constexpr size_t TRAINER_SIDEBAND_BEEP_INDEX = 8;
constexpr size_t TRAINER_SIDEBAND_AUX6_INDEX = 9;
constexpr size_t TRAINER_SIDEBAND_AUX7_INDEX = 10;
constexpr size_t TRAINER_SIDEBAND_AUX8_INDEX = 11;
constexpr size_t TRAINER_SIDEBAND_TAKEOVER_INDEX = 12;
constexpr size_t TRAINER_SIDEBAND_AIR_INDEX = 13;
constexpr size_t TRAINER_SIDEBAND_RUN_INDEX = 14;
constexpr size_t TRAINER_SIDEBAND_AUX5_INDEX = 15;
constexpr uint16_t TRAINER_IDLE_MIN_US = 1200;
constexpr uint16_t TRAINER_IDLE_MAX_US = 1275;
constexpr uint16_t EXECUTE_HIGH_US = 1700;
constexpr uint16_t AUX_OFF_US = 1000;
constexpr uint16_t TRAINER_HEARTBEAT_EDGE_US = 24;
constexpr uint32_t TRAINER_HEARTBEAT_MAX_EDGE_MS = 150;
constexpr uint32_t TRAINER_HEARTBEAT_TIMEOUT_MS = 300;
constexpr uint8_t TRAINER_HEARTBEAT_REQUIRED_EDGES = 3;

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

bool isTrainerMarker(uint16_t value)
{
  return (value >= 1200 && value < 1450) ||
         (value >= 1575 && value <= 1625) ||
         (value >= 1775 && value <= 1825) ||
         (value >= 1975 && value <= 2025);
}

void updateTrainerHeartbeat(DroneProtoCommandState& state, uint16_t marker, uint32_t nowMs)
{
  state.trainerMarkerUs = marker;
  if(!isTrainerMarker(marker))
  {
    state.trainerHeartbeatReferenceUs = 0;
    state.trainerHeartbeatLastTransitionMs = 0;
    state.trainerHeartbeatTransitions = 0;
    state.trainerHeartbeatFresh = false;
    return;
  }

  if(state.trainerHeartbeatReferenceUs == 0)
  {
    state.trainerHeartbeatReferenceUs = marker;
    state.trainerHeartbeatFresh = false;
    return;
  }

  const uint16_t delta = marker > state.trainerHeartbeatReferenceUs
    ? marker - state.trainerHeartbeatReferenceUs
    : state.trainerHeartbeatReferenceUs - marker;
  if(delta >= TRAINER_HEARTBEAT_EDGE_US)
  {
    const bool edgeOnTime = state.trainerHeartbeatLastTransitionMs != 0 &&
      nowMs - state.trainerHeartbeatLastTransitionMs <= TRAINER_HEARTBEAT_MAX_EDGE_MS;
    if(edgeOnTime)
    {
      if(state.trainerHeartbeatTransitions < TRAINER_HEARTBEAT_REQUIRED_EDGES)
        state.trainerHeartbeatTransitions++;
    }
    else
    {
      state.trainerHeartbeatTransitions = 1;
    }
    state.trainerHeartbeatReferenceUs = marker;
    state.trainerHeartbeatLastTransitionMs = nowMs;
  }

  const bool recentEdge = state.trainerHeartbeatLastTransitionMs != 0 &&
    nowMs - state.trainerHeartbeatLastTransitionMs <= TRAINER_HEARTBEAT_TIMEOUT_MS;
  state.trainerHeartbeatFresh = recentEdge &&
    state.trainerHeartbeatTransitions >= TRAINER_HEARTBEAT_REQUIRED_EDGES;
  if(!recentEdge) state.trainerHeartbeatTransitions = 0;
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

  for(size_t i = 0; i < DRONE_PROTO_FUNCTION_CHANNELS; i++)
  {
    const size_t channel = TRAINER_ROLL_INDEX + i;
    state.functionUs[i] = channel < channelCount ? transportChannels[channel] : 1500;
  }

  const uint16_t marker = TRAINER_MARKER_INDEX < channelCount
    ? transportChannels[TRAINER_MARKER_INDEX]
    : 1000;
  updateTrainerHeartbeat(state, marker, nowMs);
  const bool trainerFrameComplete = trainerSidebandChannels != nullptr &&
    trainerSidebandChannelCount >= 16;
  const bool trainerActive = !directActive && channelCount > TRAINER_MARKER_INDEX &&
    state.trainerHeartbeatFresh && trainerSidebandFresh && trainerFrameComplete;
  const DroneProtoInputSource nextSource = directActive
    ? DroneProtoInputSource::WIFI_DIRECT
    : (trainerActive ? DroneProtoInputSource::TRAINER_PHONE : DroneProtoInputSource::RADIOMASTER);

  if(nextSource != state.source)
  {
    state.source = nextSource;
    state.executeInitialized = false;
    state.executeHigh = false;
    state.trainerTaskArmed = false;
    state.selected = DroneProtoTaskCommand::NONE;
  }

  if(trainerActive)
  {
    // Trainer takeover is atomic. RadioMaster still supplies the independent
    // live heartbeat and receiver failsafe, but every logical RC value comes
    // from the same complete phone frame. If either path is stale, this block
    // is skipped and the already-mapped RadioMaster frame remains untouched.
    for(size_t i = 0; i < channelCount; i++) logicalChannels[i] = AUX_OFF_US;
    const auto copySideband = [&](size_t logicalIndex, size_t sidebandIndex)
    {
      if(logicalIndex < channelCount && sidebandIndex < trainerSidebandChannelCount)
        logicalChannels[logicalIndex] = trainerSidebandChannels[sidebandIndex];
    };
    copySideband(0, TRAINER_SIDEBAND_ROLL_INDEX);
    copySideband(1, TRAINER_SIDEBAND_PITCH_INDEX);
    copySideband(2, TRAINER_SIDEBAND_YAW_INDEX);
    copySideband(3, TRAINER_SIDEBAND_THROTTLE_INDEX);
    copySideband(4, TRAINER_SIDEBAND_ARM_INDEX);          // AUX1: arm.
    copySideband(5, TRAINER_SIDEBAND_MODE_INDEX);         // AUX2: flight mode.
    copySideband(6, TRAINER_SIDEBAND_AIR_INDEX);          // AUX3: air mode.
    copySideband(7, TRAINER_SIDEBAND_SERVO_INDEX);        // AUX4: servo.
    copySideband(8, TRAINER_SIDEBAND_TASK_INDEX);         // AUX5: task selector.
    copySideband(9, TRAINER_SIDEBAND_RUN_INDEX);          // AUX6: task run.
    copySideband(10, TRAINER_SIDEBAND_AUX5_INDEX);        // AUX7: phone Aux5.
    copySideband(11, TRAINER_SIDEBAND_AUX6_INDEX);        // AUX8: phone Aux6.
    copySideband(12, TRAINER_SIDEBAND_AUX7_INDEX);        // AUX9: phone Aux7.
    copySideband(13, TRAINER_SIDEBAND_AUX8_INDEX);        // AUX10: phone Aux8.
    copySideband(14, TRAINER_SIDEBAND_BEEP_INDEX);        // AUX11: beeper.
    copySideband(15, TRAINER_SIDEBAND_TAKEOVER_INDEX);    // AUX12: takeover.

    for(size_t i = 0; i < DRONE_PROTO_FUNCTION_CHANNELS; i++)
    {
      const size_t logicalIndex = 10 + i;
      state.functionUs[i] = logicalIndex < channelCount ? logicalChannels[logicalIndex] : AUX_OFF_US;
    }

    state.selected = decodeTrainerTask(marker);
    if(marker >= TRAINER_IDLE_MIN_US && marker < TRAINER_IDLE_MAX_US)
    {
      state.trainerTaskArmed = true;
    }
    else if(state.selected != DroneProtoTaskCommand::NONE && state.trainerTaskArmed)
    {
      queueRequest(state, state.selected, nowMs);
      state.trainerTaskArmed = false;
    }
    return;
  }

  state.trainerTaskArmed = false;
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
