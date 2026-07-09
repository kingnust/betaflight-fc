#pragma once

#include "Model.h"

#if defined(ESPFC_TARGET_DRONE_PROTO) && !defined(ESPFC_DRONE_PROTO_DISABLE_AUX)
#if defined(ESPFC_DRONE_PROTO_ENABLE_VL53L1X)
#define ESPFC_DRONE_PROTO_AUX_VL53L1X
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_PMW3901)
#define ESPFC_DRONE_PROTO_AUX_PMW3901
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_MTF02P)
#define ESPFC_DRONE_PROTO_AUX_MTF02P
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_TCS34725)
#define ESPFC_DRONE_PROTO_AUX_TCS34725
#endif
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X) || defined(ESPFC_DRONE_PROTO_AUX_PMW3901) || defined(ESPFC_DRONE_PROTO_AUX_MTF02P) || defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
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
    bool beginOpticalFlow(uint32_t now);

    Device::OpticalFlowPMW3901 _flow;
    uint32_t _lastFlowMs = 0;
    uint32_t _lastFlowInitMs = 0;
#endif
#if defined(ESPFC_DRONE_PROTO_AUX_MTF02P)
    bool beginMtf02p(uint32_t now);
    bool updateMtf02p(uint32_t now);
    void resetMtf02pParser();
    bool parseMtf02pByte(uint8_t value, uint32_t now);
    bool handleMtf02pFrame(uint32_t now);

    uint8_t _mtf02pFrame[70] = {};
    uint8_t _mtf02pFrameIndex = 0;
    uint8_t _mtf02pFrameLength = 0;
    uint8_t _mtf02pChecksum = 0;
    bool _mtf02pStarted = false;
#endif
#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
    bool beginRangefinder(uint32_t now);
    bool rangefinderFailure(uint8_t status, uint32_t now);
    void recordRangefinderError(uint8_t status, uint32_t now);
    void stopRangefinder(uint8_t status, uint32_t now, bool retry);
    bool updateRangefinder(uint32_t now);
    static void rangefinderTaskEntry(void* arg);
    void rangefinderTask();

    VL53L1X _range;
    TaskHandle_t _rangeTask = nullptr;
    uint32_t _lastRangeInitMs = 0;
    uint32_t _rangeStartedAtMs = 0;
    uint8_t _rangeFailures = 0;
    bool _rangeStarted = false;
#endif
#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
    Device::ColorTCS34725 _color;
    uint32_t _lastColorMs = 0;
    uint32_t _lastColorPrintMs = 0;
#endif
};

} // namespace Espfc::Sensor
