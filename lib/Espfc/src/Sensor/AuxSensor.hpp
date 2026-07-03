#pragma once

#include "Model.h"

#if defined(ESPFC_TARGET_DRONE_PROTO) && !defined(ESPFC_DRONE_PROTO_DISABLE_AUX)
#if defined(ESPFC_DRONE_PROTO_ENABLE_VL53L1X) || defined(ESPFC_DRONE_PROTO_ENABLE_PMW3901) || defined(ESPFC_DRONE_PROTO_ENABLE_TCS34725)
#if defined(ESPFC_DRONE_PROTO_ENABLE_VL53L1X)
#define ESPFC_DRONE_PROTO_AUX_VL53L1X
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_PMW3901)
#define ESPFC_DRONE_PROTO_AUX_PMW3901
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_TCS34725)
#define ESPFC_DRONE_PROTO_AUX_TCS34725
#endif
#else
#define ESPFC_DRONE_PROTO_AUX_VL53L1X
#define ESPFC_DRONE_PROTO_AUX_PMW3901
#define ESPFC_DRONE_PROTO_AUX_TCS34725
#endif
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X) || defined(ESPFC_DRONE_PROTO_AUX_PMW3901) || defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
#define ESPFC_DRONE_PROTO_AUX_ENABLED
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
#include "Device/ColorTCS34725.hpp"
#endif
#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
#include "Device/OpticalFlow/OpticalFlowPMW3901.hpp"
#endif
#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
    Device::OpticalFlowPMW3901 _flow;
    uint32_t _lastFlowMs = 0;
#endif
#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
    void beginRangefinder(uint32_t now);
    bool rangefinderFailure(uint8_t status, uint32_t now);
    static void rangefinderTaskEntry(void* arg);
    void rangefinderTask();
    bool updateRangefinder(uint32_t now);

    VL53L1X _range;
    TaskHandle_t _rangeTask = nullptr;
    uint32_t _lastRangeInitMs = 0;
    uint32_t _lastRangeMs = 0;
    uint8_t _rangeFailures = 0;
    bool _rangeStarted = false;
    bool _rangeBusStarted = false;
#endif
#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
    Device::ColorTCS34725 _color;
    uint32_t _lastColorMs = 0;
#endif
};

} // namespace Espfc::Sensor
