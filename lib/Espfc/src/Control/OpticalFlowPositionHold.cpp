#include "Control/OpticalFlowPositionHold.h"

#include "Control/DroneProtoCommandRouter.hpp"
#include "Model.h"
#include "Utils/Math.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Control {

namespace {

constexpr uint32_t SENSOR_MAX_AGE_MS = 150;
constexpr uint16_t RANGE_MIN_MM = 80;
constexpr uint16_t RANGE_MAX_MM = 4000;
constexpr uint8_t MTF_MIN_FLOW_QUALITY = 30;
constexpr float MAX_TILT_RAD = 0.436332f;             // 25 degrees.
constexpr float PILOT_DEADBAND = 0.08f;
constexpr float PILOT_MAX_VELOCITY_MPS = 1.0f;
constexpr float POSITION_GAIN = 0.8f;
constexpr float POSITION_MAX_VELOCITY_MPS = 0.6f;
constexpr float VELOCITY_TO_ANGLE = 0.14f;
constexpr float MAX_CORRECTION_ANGLE_RAD = 0.139626f; // 8 degrees.
constexpr float FLOW_FILTER_ALPHA = 0.30f;
constexpr float PRESET_DISTANCE_M = 1.0f;
constexpr uint32_t POSITION_TASK_MAX_AGE_MS = 500;

#ifndef ESPFC_DRONE_PROTO_FLOW_FORWARD_SIGN
#define ESPFC_DRONE_PROTO_FLOW_FORWARD_SIGN 1
#endif

#ifndef ESPFC_DRONE_PROTO_FLOW_RIGHT_SIGN
#define ESPFC_DRONE_PROTO_FLOW_RIGHT_SIGN 1
#endif

bool isFresh(uint32_t updatedAt, uint32_t nowMs)
{
  if(updatedAt == 0) return false;
  const int32_t age = static_cast<int32_t>(nowMs - updatedAt);
  return age < 0 ? (updatedAt - nowMs) <= SENSOR_MAX_AGE_MS : static_cast<uint32_t>(age) <= SENSOR_MAX_AGE_MS;
}

float applyDeadband(float value)
{
  if(std::fabs(value) <= PILOT_DEADBAND) return 0.0f;
  const float magnitude = (std::fabs(value) - PILOT_DEADBAND) / (1.0f - PILOT_DEADBAND);
  return std::copysign(std::min(magnitude, 1.0f), value);
}

bool isPositionTask(DroneProtoTaskCommand command)
{
  return command == DroneProtoTaskCommand::GO_TO_PRESET_1 ||
         command == DroneProtoTaskCommand::GO_TO_PRESET_2 ||
         command == DroneProtoTaskCommand::RETURN_HOME;
}

}

PositionHoldFault positionHoldSensorFault(const Model& model, uint32_t nowMs)
{
  if(!model.accelActive()) return POSHOLD_NO_ACCEL;
  if(!model.baroActive()) return POSHOLD_NO_BARO;
  if(!model.rangefinderActive()) return POSHOLD_NO_RANGE;
  if(!isFresh(model.state.aux.range.lastUpdate, nowMs)) return POSHOLD_RANGE_STALE;
  if(model.state.aux.range.distanceMm < RANGE_MIN_MM || model.state.aux.range.distanceMm > RANGE_MAX_MM)
    return POSHOLD_RANGE_INVALID;
  if(!model.opticalFlowActive()) return POSHOLD_NO_FLOW;
#if !defined(ESPFC_DRONE_PROTO_ENABLE_MTF02P)
  // PMW3901 reports frame displacement counts, not the calibrated cm/s@1m
  // velocity consumed by this controller. Keep the sensor available for
  // diagnostics without enabling an incorrectly scaled flight mode.
  return POSHOLD_FLOW_UNCALIBRATED;
#endif
  if(!isFresh(model.state.aux.flow.lastUpdate, nowMs)) return POSHOLD_FLOW_STALE;
#if defined(ESPFC_DRONE_PROTO_ENABLE_MTF02P)
  if(model.state.aux.mtf02p.present && model.state.aux.mtf02p.flowQuality < MTF_MIN_FLOW_QUALITY)
    return POSHOLD_FLOW_QUALITY;
#endif
  const float tilt = std::max(std::fabs(model.state.attitude.euler[AXIS_ROLL]),
                              std::fabs(model.state.attitude.euler[AXIS_PITCH]));
  if(tilt > MAX_TILT_RAD) return POSHOLD_TILT;
  return POSHOLD_OK;
}

const char * positionHoldFaultName(PositionHoldFault fault)
{
  switch(fault)
  {
    case POSHOLD_OK: return "OK";
    case POSHOLD_NO_ACCEL: return "NO_ACCEL";
    case POSHOLD_NO_BARO: return "NO_BARO";
    case POSHOLD_NO_RANGE: return "NO_RANGE";
    case POSHOLD_RANGE_STALE: return "RANGE_STALE";
    case POSHOLD_RANGE_INVALID: return "RANGE_INVALID";
    case POSHOLD_NO_FLOW: return "NO_FLOW";
    case POSHOLD_FLOW_UNCALIBRATED: return "FLOW_UNCALIBRATED";
    case POSHOLD_FLOW_STALE: return "FLOW_STALE";
    case POSHOLD_FLOW_QUALITY: return "FLOW_QUALITY";
    case POSHOLD_TILT: return "TILT";
    default: return "UNKNOWN";
  }
}

OpticalFlowPositionHold::OpticalFlowPositionHold(Model& model): _model(model) {}

int OpticalFlowPositionHold::begin()
{
  resetLocalPosition();
  _model.state.positionHold.healthy = false;
  _model.state.positionHold.fault = POSHOLD_NO_FLOW;
  return 1;
}

void OpticalFlowPositionHold::resetLocalPosition()
{
  auto& state = _model.state.positionHold;
  state.active = false;
  state.lastFlowUpdateMs = 0;
  state.lastFlowFrame = 0;
  state.velocityBody[0] = state.velocityBody[1] = 0.0f;
  state.positionEarth[0] = state.positionEarth[1] = 0.0f;
  state.targetEarth[0] = state.targetEarth[1] = 0.0f;
  state.angleSetpoint[0] = state.angleSetpoint[1] = 0.0f;
  _filteredVelocity[0] = _filteredVelocity[1] = 0.0f;
  state.resetCount++;
}

void OpticalFlowPositionHold::updateFlowMeasurement()
{
  auto& state = _model.state.positionHold;
  const auto& flow = _model.state.aux.flow;
  if(flow.frameCount == state.lastFlowFrame || flow.lastUpdate == 0) return;

  float dt = 0.02f;
  if(state.lastFlowUpdateMs != 0)
  {
    dt = std::clamp((flow.lastUpdate - state.lastFlowUpdateMs) * 0.001f, 0.005f, 0.10f);
  }
  state.lastFlowUpdateMs = flow.lastUpdate;
  state.lastFlowFrame = flow.frameCount;

  const float heightM = _model.state.aux.range.distanceMm * 0.001f;
  const float measuredForward = flow.deltaX * heightM * 0.01f * ESPFC_DRONE_PROTO_FLOW_FORWARD_SIGN;
  const float measuredRight = flow.deltaY * heightM * 0.01f * ESPFC_DRONE_PROTO_FLOW_RIGHT_SIGN;
  _filteredVelocity[0] += (measuredForward - _filteredVelocity[0]) * FLOW_FILTER_ALPHA;
  _filteredVelocity[1] += (measuredRight - _filteredVelocity[1]) * FLOW_FILTER_ALPHA;
  state.velocityBody[0] = _filteredVelocity[0];
  state.velocityBody[1] = _filteredVelocity[1];

  const float yaw = _model.state.attitude.euler[AXIS_YAW];
  const float cosine = std::cos(yaw);
  const float sine = std::sin(yaw);
  const float earthVelocityX = cosine * state.velocityBody[0] - sine * state.velocityBody[1];
  const float earthVelocityY = sine * state.velocityBody[0] + cosine * state.velocityBody[1];
  state.positionEarth[0] += earthVelocityX * dt;
  state.positionEarth[1] += earthVelocityY * dt;
}

void OpticalFlowPositionHold::expirePendingPositionTask(uint32_t nowMs)
{
  auto& pending = _model.state.commands.pending;
  if(!pending.valid || !isPositionTask(pending.command)) return;
  if(static_cast<uint32_t>(nowMs - pending.receivedAtMs) > POSITION_TASK_MAX_AGE_MS)
  {
    pending.valid = false;
  }
}

void OpticalFlowPositionHold::applyPendingPositionTask(uint32_t nowMs)
{
  auto& commands = _model.state.commands;
  if(!commands.pending.valid) return;
  if(static_cast<uint32_t>(nowMs - commands.pending.receivedAtMs) > POSITION_TASK_MAX_AGE_MS)
  {
    if(isPositionTask(commands.pending.command)) commands.pending.valid = false;
    return;
  }

  auto& hold = _model.state.positionHold;
  switch(commands.pending.command)
  {
    case DroneProtoTaskCommand::GO_TO_PRESET_1:
      hold.targetEarth[0] = PRESET_DISTANCE_M;
      hold.targetEarth[1] = 0.0f;
      break;
    case DroneProtoTaskCommand::GO_TO_PRESET_2:
      hold.targetEarth[0] = 0.0f;
      hold.targetEarth[1] = PRESET_DISTANCE_M;
      break;
    case DroneProtoTaskCommand::RETURN_HOME:
      hold.targetEarth[0] = 0.0f;
      hold.targetEarth[1] = 0.0f;
      break;
    default:
      return;
  }

  hold.lastTaskSequence = commands.pending.sequence;
  commands.pending.valid = false;
}

bool OpticalFlowPositionHold::update()
{
  auto& state = _model.state.positionHold;
  const uint32_t nowMs = millis();
  const bool armed = _model.isModeActive(MODE_ARMED);
  const bool requested = _model.isModeActive(MODE_POSHOLD);
  state.requested = requested;

  if(armed && !_wasArmed) resetLocalPosition();
  _wasArmed = armed;

  expirePendingPositionTask(nowMs);
  state.fault = positionHoldSensorFault(_model, nowMs);
  state.healthy = state.fault == POSHOLD_OK;
  if(state.healthy && armed) updateFlowMeasurement();

  if(!armed || !requested || !state.healthy)
  {
    state.active = false;
    state.angleSetpoint[0] = state.angleSetpoint[1] = 0.0f;
    _wasRequested = requested;
    return false;
  }

  if(!_wasRequested || !state.active)
  {
    state.targetEarth[0] = state.positionEarth[0];
    state.targetEarth[1] = state.positionEarth[1];
  }
  _wasRequested = true;
  state.active = true;
  applyPendingPositionTask(nowMs);

  const float pilotForward = applyDeadband(_model.state.input.ch[AXIS_PITCH]);
  const float pilotRight = applyDeadband(_model.state.input.ch[AXIS_ROLL]);
  float desiredBodyForward = 0.0f;
  float desiredBodyRight = 0.0f;

  if(pilotForward != 0.0f || pilotRight != 0.0f)
  {
    desiredBodyForward = pilotForward * PILOT_MAX_VELOCITY_MPS;
    desiredBodyRight = pilotRight * PILOT_MAX_VELOCITY_MPS;
    state.targetEarth[0] = state.positionEarth[0];
    state.targetEarth[1] = state.positionEarth[1];
  }
  else
  {
    const float desiredEarthX = std::clamp((state.targetEarth[0] - state.positionEarth[0]) * POSITION_GAIN,
                                           -POSITION_MAX_VELOCITY_MPS, POSITION_MAX_VELOCITY_MPS);
    const float desiredEarthY = std::clamp((state.targetEarth[1] - state.positionEarth[1]) * POSITION_GAIN,
                                           -POSITION_MAX_VELOCITY_MPS, POSITION_MAX_VELOCITY_MPS);
    const float yaw = _model.state.attitude.euler[AXIS_YAW];
    const float cosine = std::cos(yaw);
    const float sine = std::sin(yaw);
    desiredBodyForward = cosine * desiredEarthX + sine * desiredEarthY;
    desiredBodyRight = -sine * desiredEarthX + cosine * desiredEarthY;
  }

  const float pitch = (desiredBodyForward - state.velocityBody[0]) * VELOCITY_TO_ANGLE;
  const float roll = (desiredBodyRight - state.velocityBody[1]) * VELOCITY_TO_ANGLE;
  state.angleSetpoint[AXIS_ROLL] = std::clamp(roll, -MAX_CORRECTION_ANGLE_RAD, MAX_CORRECTION_ANGLE_RAD);
  state.angleSetpoint[AXIS_PITCH] = std::clamp(pitch, -MAX_CORRECTION_ANGLE_RAD, MAX_CORRECTION_ANGLE_RAD);
  return true;
}

}
