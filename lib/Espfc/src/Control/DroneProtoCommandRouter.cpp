#include "Control/DroneProtoCommandRouter.hpp"

namespace Espfc {

namespace Control {

namespace {

constexpr size_t RADIO_TASK_SELECT_INDEX = 8;   // Physical CH9.
constexpr size_t RADIO_TASK_EXECUTE_INDEX = 9;  // Physical CH10.
constexpr size_t TRAINER_ROLL_INDEX = 10;       // Physical CH11 through CH15.
constexpr size_t TRAINER_MARKER_INDEX = 15;     // Physical CH16.
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
  const bool trainerActive = !directActive && channelCount > TRAINER_MARKER_INDEX &&
    state.trainerHeartbeatFresh;
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
    // Phone controls arrive on physical CH11-CH15. Convert AETR transport
    // order into ESP-FC's R/P/Y/T/AUX1 logical order and suppress every other
    // radio auxiliary command while the live CH16 heartbeat is present. CH6
    // remains the three-position flight mode and CH8 remains the positional
    // servo channel; EdgeTX can replace both from trainer TR8/TR7.
    if(channelCount >= TRAINER_ROLL_INDEX + 5)
    {
      logicalChannels[0] = transportChannels[10];
      logicalChannels[1] = transportChannels[11];
      logicalChannels[2] = transportChannels[13];
      logicalChannels[3] = transportChannels[12];
      logicalChannels[4] = transportChannels[14];
    }
    for(size_t i = 5; i < channelCount; i++) logicalChannels[i] = AUX_OFF_US;
    if(channelCount > 5) logicalChannels[5] = transportChannels[5];
    if(channelCount > 7) logicalChannels[7] = transportChannels[7];

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
