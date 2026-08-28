#pragma once

#include "Control/Altitude.hpp"
#include "Control/ChirpExcitation.hpp"
#include "Control/OpticalFlowPositionHold.h"
#include "Control/Rates.h"
#include "Model.h"

namespace Espfc::Control {

class Controller
{
public:
  Controller(Model& model);
  int begin();
  int update();

  void outerLoopRobot();
  void innerLoopRobot();
  void outerLoop();
  void innerLoop();

  inline float getTpaFactor() const;
  inline void resetIterm();
  float calculateSetpointRate(int axis, float input) const;
  float calcualteAltHoldSetpoint() const;

private:
  void updateChirp();
  void clearChirpDebug();
  bool chirpSafetyReady() const;
  void beginAltHold();
  void beginInnerLoop(size_t axis);
  void beginOuterLoop(size_t axis);

  Model& _model;
  Rates _rates;
  Utils::Filter _speedFilter;
  OpticalFlowPositionHold _positionHold;
  ChirpExcitation _chirp;
  uint8_t _chirpAxis = AXIS_ROLL;
  bool _chirpSwitchWasActive = false;
  bool _chirpRunning = false;
  bool _chirpAdvanceOnRelease = false;
};

} // namespace Espfc::Control
