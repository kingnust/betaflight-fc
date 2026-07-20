#pragma once

#include <stdint.h>

namespace Espfc {

class Model;

namespace Control {

enum PositionHoldFault : uint8_t
{
  POSHOLD_OK = 0,
  POSHOLD_NO_ACCEL,
  POSHOLD_NO_BARO,
  POSHOLD_NO_RANGE,
  POSHOLD_RANGE_STALE,
  POSHOLD_RANGE_INVALID,
  POSHOLD_NO_FLOW,
  POSHOLD_FLOW_UNCALIBRATED,
  POSHOLD_FLOW_STALE,
  POSHOLD_FLOW_QUALITY,
  POSHOLD_TILT,
};

struct OpticalFlowPositionHoldState
{
  bool requested = false;
  bool active = false;
  bool healthy = false;
  PositionHoldFault fault = POSHOLD_NO_FLOW;
  uint32_t lastFlowUpdateMs = 0;
  uint32_t lastFlowFrame = 0;
  uint32_t resetCount = 0;
  uint32_t lastTaskSequence = 0;
  float velocityBody[2] = {0.0f, 0.0f};
  float positionEarth[2] = {0.0f, 0.0f};
  float targetEarth[2] = {0.0f, 0.0f};
  float angleSetpoint[2] = {0.0f, 0.0f};
};

PositionHoldFault positionHoldSensorFault(const Model& model, uint32_t nowMs);
const char * positionHoldFaultName(PositionHoldFault fault);

class OpticalFlowPositionHold
{
  public:
    explicit OpticalFlowPositionHold(Model& model);
    int begin();
    bool update();

  private:
    void resetLocalPosition();
    void updateFlowMeasurement();
    void expirePendingPositionTask(uint32_t nowMs);
    void applyPendingPositionTask(uint32_t nowMs);

    Model& _model;
    bool _wasArmed = false;
    bool _wasRequested = false;
    float _filteredVelocity[2] = {0.0f, 0.0f};
};

}

}
