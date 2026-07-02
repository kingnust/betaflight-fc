#pragma once

#include "Model.h"

#if defined(ESPFC_TARGET_DRONE_PROTO) && !defined(ESPFC_DRONE_PROTO_DISABLE_AUX)
#define ESPFC_DRONE_PROTO_AUX_ENABLED
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
#include "Device/ColorTCS34725.hpp"
#include "Device/OpticalFlow/OpticalFlowPMW3901.hpp"
#include <VL53L1X.h>
#include <Wire.h>
#endif

namespace Espfc::Sensor {

class AuxSensor
{
  public:
    AuxSensor(Model& model);

    int begin();
    int update();

  private:
    Model& _model;

#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
    Device::OpticalFlowPMW3901 _flow;
    Device::ColorTCS34725 _color;
    VL53L1X _range;
    uint32_t _lastFlowMs = 0;
    uint32_t _lastRangeMs = 0;
    uint32_t _lastColorMs = 0;
    bool _rangeStarted = false;
#endif
};

} // namespace Espfc::Sensor
