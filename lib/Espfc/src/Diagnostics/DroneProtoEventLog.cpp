#include "Diagnostics/DroneProtoEventLog.hpp"

#include "Device/DroneProtoServo.hpp"
#include "Model.h"
#include <cmath>

namespace Espfc::Diagnostics {

DroneProtoEventLog::DroneProtoEventLog(Model& model): _model(model) {}

void DroneProtoEventLog::begin()
{
  auto& state = _model.state;
  _source = static_cast<uint8_t>(state.commands.source);
  _trainerRequest = state.commands.trainerTakeoverRequested;
  _trainerActive = state.commands.trainerTakeoverLatched;
  _armingFlags = state.mode.armingDisabledFlags;
  _receiverHealthy = state.input.channelsValid && !state.input.rxLoss && !state.input.rxFailSafe;
  _failsafe = static_cast<uint8_t>(state.failsafe.phase);
  _posholdRelease = static_cast<uint8_t>(state.positionHold.release);
  _nextSampleMs = millis();
}

void DroneProtoEventLog::push(EventType type, int32_t a, int32_t b, int32_t c, uint32_t nowMs)
{
  auto& log = _model.state.eventLog;
  log.entries[log.next] = {nowMs, type, a, b, c};
  log.next = (log.next + 1) % EVENT_LOG_CAPACITY;
  if(log.count < EVENT_LOG_CAPACITY) log.count++;
  else log.dropped++;
}

void DroneProtoEventLog::update(uint32_t nowMs)
{
  auto& state = _model.state;
  const uint8_t source = static_cast<uint8_t>(state.commands.source);
  const bool trainerRequest = state.commands.trainerTakeoverRequested;
  const bool trainerActive = state.commands.trainerTakeoverLatched;
  const uint32_t armingFlags = state.mode.armingDisabledFlags;
  const bool receiverHealthy = state.input.channelsValid && !state.input.rxLoss && !state.input.rxFailSafe;
  const uint8_t failsafe = static_cast<uint8_t>(state.failsafe.phase);
  const uint8_t posholdRelease = static_cast<uint8_t>(state.positionHold.release);

  if(source != _source) { push(EventType::CONTROL_SOURCE, source, _source, 0, nowMs); _source = source; }
  if(trainerRequest != _trainerRequest) { push(EventType::TRAINER_REQUEST, trainerRequest, state.commands.trainerLinkQualified, 0, nowMs); _trainerRequest = trainerRequest; }
  if(trainerActive != _trainerActive) { push(EventType::TRAINER_ACTIVE, trainerActive, state.commands.trainerTakeoverBlockedArmed, state.commands.trainerTakeoverBlockedTrainerArmed, nowMs); _trainerActive = trainerActive; }
  if(armingFlags != _armingFlags) { push(EventType::ARMING_FLAGS, armingFlags, _armingFlags, 0, nowMs); _armingFlags = armingFlags; }
  if(receiverHealthy != _receiverHealthy) { push(EventType::RECEIVER_HEALTH, receiverHealthy, state.input.rxLoss, state.input.rxFailSafe, nowMs); _receiverHealthy = receiverHealthy; }
  if(failsafe != _failsafe) { push(EventType::FAILSAFE, failsafe, _failsafe, 0, nowMs); _failsafe = failsafe; }
  if(posholdRelease != _posholdRelease) { push(EventType::POSHOLD_RELEASE, posholdRelease, state.positionHold.releaseLatched, 0, nowMs); _posholdRelease = posholdRelease; }

  if(static_cast<int32_t>(nowMs - _nextSampleMs) >= 0)
  {
    const int32_t altitudeCm = std::lround(state.altitude.height * 100.0f);
    const int32_t packedHealth = (receiverHealthy ? 1 : 0) |
      (state.positionHold.active ? 2 : 0) |
      (static_cast<int32_t>(state.aux.mtf02p.flowQuality) << 8);
    push(EventType::SAMPLE, altitudeCm, Device::DroneProtoServo::currentUs(), packedHealth, nowMs);
    _nextSampleMs = nowMs + 1000;
  }
}

const char * DroneProtoEventLog::typeName(EventType type)
{
  switch(type)
  {
    case EventType::SAMPLE: return "SAMPLE";
    case EventType::CONTROL_SOURCE: return "SOURCE";
    case EventType::TRAINER_REQUEST: return "TRAINER_REQUEST";
    case EventType::TRAINER_ACTIVE: return "TRAINER_ACTIVE";
    case EventType::ARMING_FLAGS: return "ARMING_FLAGS";
    case EventType::RECEIVER_HEALTH: return "RX_HEALTH";
    case EventType::FAILSAFE: return "FAILSAFE";
    case EventType::POSHOLD_RELEASE: return "POSHOLD_RELEASE";
    default: return "UNKNOWN";
  }
}

}
