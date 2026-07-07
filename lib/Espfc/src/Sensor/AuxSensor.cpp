#include "Sensor/AuxSensor.hpp"

#include "Hardware.h"
#include <cmath>

#if defined(ESPFC_DRONE_PROTO_SONAR_DEBUG)
#define SONAR_DEBUG_LINE(v) do { Serial0.print("SONAR DEBUG: "); Serial0.println(v); Serial0.flush(); } while(0)
#define SONAR_DEBUG_VALUE(k, v) do { Serial0.print("SONAR DEBUG: "); Serial0.print(k); Serial0.print("="); Serial0.println(v); Serial0.flush(); } while(0)
#define SONAR_DEBUG_HEX(k, v) do { Serial0.print("SONAR DEBUG: "); Serial0.print(k); Serial0.print("=0x"); Serial0.println(v, HEX); Serial0.flush(); } while(0)
#else
#define SONAR_DEBUG_LINE(v)
#define SONAR_DEBUG_VALUE(k, v)
#define SONAR_DEBUG_HEX(k, v)
#endif

#if !defined(ESPFC_DRONE_PROTO_SONAR_DEBUG)
#undef SONAR_DEBUG_LINE
#undef SONAR_DEBUG_VALUE
#undef SONAR_DEBUG_HEX
#define SONAR_DEBUG_LINE(v) do { yield(); vTaskDelay(pdMS_TO_TICKS(10)); } while(0)
#define SONAR_DEBUG_VALUE(k, v) do { yield(); vTaskDelay(pdMS_TO_TICKS(10)); } while(0)
#define SONAR_DEBUG_HEX(k, v) do { yield(); vTaskDelay(pdMS_TO_TICKS(10)); } while(0)
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
namespace {

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
constexpr uint32_t PMW3901_BOOT_DELAY_MS = 1000;
constexpr uint32_t PMW3901_RETRY_MS = 2000;
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
TwoWire& rangefinderWire()
{
  return Wire;
}

constexpr uint32_t VL53L1X_I2C_SPEED = 100000;
constexpr uint8_t VL53L1X_ADDRESS = 0x29;
constexpr uint16_t VL53L1X_MODEL_ID = 0xEACC;
constexpr uint16_t VL53L1X_MODEL_ID_COMPAT = 0xEAAA;
constexpr uint32_t VL53L1X_BOOT_DELAY_MS = 1500;
constexpr uint32_t VL53L1X_INIT_RETRY_MS = 3000;
constexpr uint16_t VL53L1X_I2C_TIMEOUT_MS = 100;
constexpr uint16_t VL53L1X_SENSOR_TIMEOUT_MS = 500;
constexpr uint32_t VL53L1X_UPDATE_MS = 500;
constexpr uint32_t VL53L1X_MEASUREMENT_BUDGET_US = 50000;
constexpr uint8_t VL53L1X_MAX_FAILURES = 4;
constexpr uint32_t VL53L1X_TASK_STACK_BYTES = 8192;
constexpr uint32_t VL53L1X_RETRY_DISABLED = 0xffffffffu;

bool rangefinderAddressPresent(TwoWire& bus)
{
  SONAR_DEBUG_LINE("address probe begin");
  bus.beginTransmission(VL53L1X_ADDRESS);
  const uint8_t status = bus.endTransmission();
  SONAR_DEBUG_VALUE("address probe status", status);
  return status == 0;
}

bool rangefinderReadModelId(TwoWire& bus, uint16_t& modelId)
{
  SONAR_DEBUG_LINE("model id begin");
  bus.beginTransmission(VL53L1X_ADDRESS);
  bus.write((uint8_t)(VL53L1X::IDENTIFICATION__MODEL_ID >> 8));
  bus.write((uint8_t)VL53L1X::IDENTIFICATION__MODEL_ID);
  const uint8_t writeStatus = bus.endTransmission();
  SONAR_DEBUG_VALUE("model id write status", writeStatus);
  if (writeStatus != 0)
  {
    return false;
  }

  const uint8_t readCount = bus.requestFrom(VL53L1X_ADDRESS, (uint8_t)2);
  SONAR_DEBUG_VALUE("model id read count", readCount);
  if (readCount != 2)
  {
    return false;
  }

  modelId = ((uint16_t)bus.read() << 8) | bus.read();
  SONAR_DEBUG_HEX("model id", modelId);
  return true;
}
#endif

} // namespace
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X) || defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
namespace {

int16_t debugClamp(uint32_t value)
{
  return value > 32767u ? 32767 : (int16_t)value;
}

} // namespace
#endif

namespace Espfc::Sensor {

AuxSensor::AuxSensor(Model& model): _model(model) {}

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
bool AuxSensor::beginOpticalFlow(uint32_t now)
{
  _lastFlowInitMs = now + PMW3901_RETRY_MS;
  if (Device::BusSPI* spi = Device::getMainSpiBus())
  {
    _model.state.aux.flow.present = _flow.begin(spi, ESPFC_PMW3901_CS);
    _model.state.aux.flow.chipId = _flow.getChipId();
    _model.state.aux.flow.inverseChipId = _flow.getInverseChipId();
    if (_model.state.aux.flow.present)
    {
      _model.logger.info().log(F("AUX PMW3901")).logln(F("Y"));
    }
    return _model.state.aux.flow.present;
  }
  return false;
}
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
void AuxSensor::stopRangefinder(uint8_t status, uint32_t now)
{
  SONAR_DEBUG_VALUE("stop status", status);
  _rangeStarted = false;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = status;
  _model.state.aux.range.lastUpdate = 0;
  _lastRangeInitMs = VL53L1X_RETRY_DISABLED;
  _rangeFailures = 0;
}

bool AuxSensor::beginRangefinder(uint32_t now)
{
  SONAR_DEBUG_LINE("begin start");
  _lastRangeInitMs = now;
  _rangeFailures = 0;
  _rangeStarted = false;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = 255;
  _model.state.aux.range.lastUpdate = 0;
  TwoWire& bus = rangefinderWire();

  pinMode(ESPFC_VL53_I2C_SDA, INPUT_PULLUP);
  pinMode(ESPFC_VL53_I2C_SCL, INPUT_PULLUP);
  SONAR_DEBUG_LINE("before wire begin");
  bus.begin(ESPFC_VL53_I2C_SDA, ESPFC_VL53_I2C_SCL, VL53L1X_I2C_SPEED);
  SONAR_DEBUG_LINE("after wire begin");
  bus.setClock(VL53L1X_I2C_SPEED);
  SONAR_DEBUG_LINE("after set clock");
  bus.setTimeOut(VL53L1X_I2C_TIMEOUT_MS);
  SONAR_DEBUG_LINE("after set timeout");
  _rangeBusStarted = true;
  vTaskDelay(pdMS_TO_TICKS(10));

  if (!rangefinderAddressPresent(bus))
  {
    stopRangefinder(252, now);
    return false;
  }

  uint16_t modelId = 0;
  if (!rangefinderReadModelId(bus, modelId))
  {
    stopRangefinder(251, now);
    return false;
  }
  if (modelId != VL53L1X_MODEL_ID && modelId != VL53L1X_MODEL_ID_COMPAT)
  {
    stopRangefinder(250, now);
    return false;
  }

  _range.setBus(&bus);
  _range.setTimeout(VL53L1X_SENSOR_TIMEOUT_MS);
  SONAR_DEBUG_LINE("before vl init");
  if (!_range.init())
  {
    stopRangefinder(_range.timeoutOccurred() ? 248 : 249, now);
    return false;
  }
  SONAR_DEBUG_LINE("after vl init");

  SONAR_DEBUG_LINE("before distance mode");
  if (!_range.setDistanceMode(VL53L1X::Long))
  {
    stopRangefinder(247, now);
    return false;
  }
  SONAR_DEBUG_LINE("after distance mode");

  SONAR_DEBUG_LINE("before timing budget");
  if (!_range.setMeasurementTimingBudget(VL53L1X_MEASUREMENT_BUDGET_US))
  {
    stopRangefinder(246, now);
    return false;
  }
  SONAR_DEBUG_LINE("after timing budget");

  SONAR_DEBUG_LINE("before start continuous");
  _range.startContinuous(VL53L1X_UPDATE_MS);
  SONAR_DEBUG_LINE("after start continuous");
  if (_range.timeoutOccurred() || _range.last_status != 0)
  {
    stopRangefinder(245, now);
    return false;
  }

  _rangeStarted = true;
  _model.state.aux.range.present = true;
  _model.state.aux.range.status = 254;
  _lastRangeMs = now;
  _model.logger.info().log(F("AUX VL53L1X")).logln(F("Y"));
  return true;
}

bool AuxSensor::rangefinderFailure(uint8_t status, uint32_t now)
{
  _model.state.aux.range.status = status;
  _rangeFailures++;
  if (_rangeFailures >= VL53L1X_MAX_FAILURES)
  {
    stopRangefinder(status, now);
  }
  return false;
}

bool AuxSensor::updateRangefinder(uint32_t now)
{
  if (!_rangeStarted || !_model.state.aux.range.present)
  {
    return false;
  }

  _lastRangeMs = now;
  if (!_range.dataReady())
  {
    if (_range.last_status != 0)
    {
      return rangefinderFailure(244, now);
    }
    if (_model.state.aux.range.lastUpdate && now - _model.state.aux.range.lastUpdate > 2000)
    {
      _model.state.aux.range.status = 240;
    }
    return false;
  }

  SONAR_DEBUG_LINE("before read");
  const uint32_t readStart = millis();
  const uint16_t distance = _range.read(false);
  SONAR_DEBUG_VALUE("after read", distance);
  if (_range.timeoutOccurred() || _range.last_status != 0)
  {
    return rangefinderFailure(242, now);
  }
  if (millis() - readStart > VL53L1X_SENSOR_TIMEOUT_MS)
  {
    return rangefinderFailure(243, now);
  }

  _model.state.aux.range.distanceMm = distance;
  _model.state.aux.range.present = true;
  _model.state.aux.range.status = (uint8_t)_range.ranging_data.range_status;
  _model.state.aux.range.signal = debugClamp(lrintf(_range.ranging_data.peak_signal_count_rate_MCPS * 1000.0f));
  _model.state.aux.range.ambient = debugClamp(lrintf(_range.ranging_data.ambient_count_rate_MCPS * 1000.0f));
  _model.state.aux.range.lastUpdate = now;
  _rangeFailures = 0;

  if (_model.config.debug.mode == DEBUG_RANGEFINDER || _model.config.debug.mode == DEBUG_LIDAR_TF)
  {
    _model.state.debug[0] = distance;
    _model.state.debug[1] = _model.state.aux.range.status;
  }
  if (_model.config.debug.mode == DEBUG_RANGEFINDER_QUALITY)
  {
    _model.state.debug[0] = _model.state.aux.range.status;
    _model.state.debug[1] = _model.state.aux.range.signal;
    _model.state.debug[2] = _model.state.aux.range.ambient;
  }

  return true;
}

void AuxSensor::rangefinderTaskEntry(void* arg)
{
  static_cast<AuxSensor*>(arg)->rangefinderTask();
  vTaskDelete(nullptr);
}

void AuxSensor::rangefinderTask()
{
  SONAR_DEBUG_LINE("task boot delay");
  vTaskDelay(pdMS_TO_TICKS(VL53L1X_BOOT_DELAY_MS));
  SONAR_DEBUG_LINE("task start");
  _lastRangeInitMs = 0;

  for (;;)
  {
    const uint32_t now = millis();

    if (!_rangeStarted)
    {
      if (_lastRangeInitMs == VL53L1X_RETRY_DISABLED)
      {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      if (_lastRangeInitMs == 0 || now >= _lastRangeInitMs)
      {
        beginRangefinder(now);
      }
      vTaskDelay(pdMS_TO_TICKS(_rangeStarted ? VL53L1X_UPDATE_MS : 250));
      continue;
    }

    updateRangefinder(now);
    vTaskDelay(pdMS_TO_TICKS(VL53L1X_UPDATE_MS));
  }
}
#endif

int AuxSensor::begin()
{
#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901) && defined(ESPFC_SPI_0)
  _model.state.aux.flow.present = false;
  _model.state.aux.flow.chipId = 0;
  _model.state.aux.flow.inverseChipId = 0;
  _lastFlowInitMs = millis() + PMW3901_BOOT_DELAY_MS;
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725) && defined(ESPFC_I2C_0)
  if (Device::BusI2C* i2c = Device::getMainI2cBus())
  {
    _model.state.aux.color.present = _color.begin(i2c, ESPFC_TCS_LED_PIN);
    _model.state.aux.color.ledOn = _model.state.aux.color.present;
    _model.logger.info().log(F("AUX TCS34725")).logln(_model.state.aux.color.present ? "Y" : "");
  }
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
  SONAR_DEBUG_LINE("begin task create");
  _lastRangeInitMs = millis() + VL53L1X_BOOT_DELAY_MS;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = 253;
  if (!_rangeTask)
  {
    const BaseType_t started = xTaskCreate(rangefinderTaskEntry, "vl53Task", VL53L1X_TASK_STACK_BYTES, this, 0, &_rangeTask);
    SONAR_DEBUG_VALUE("task create", started);
    _model.logger.info().log(F("AUX VL53L1X")).logln(started == pdPASS ? "TASK" : "TASK FAIL");
    if (started != pdPASS)
    {
      _model.state.aux.range.status = 241;
    }
  }
#endif
#endif

  return 1;
}

int AuxSensor::update()
{
#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901) || defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
  const uint32_t now = millis();
#endif
  int updated = 0;

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
  if (!_model.state.aux.flow.present && now >= _lastFlowInitMs)
  {
    beginOpticalFlow(now);
  }

  if (_model.state.aux.flow.present && now - _lastFlowMs >= 20)
  {
    _lastFlowMs = now;
    int16_t dx = 0;
    int16_t dy = 0;
    if (_flow.readMotion(dx, dy))
    {
      _model.state.aux.flow.deltaX = dx;
      _model.state.aux.flow.deltaY = dy;
      _model.state.aux.flow.frameCount++;
      _model.state.aux.flow.lastUpdate = now;
      if (_model.config.debug.mode == DEBUG_RANGEFINDER || _model.config.debug.mode == DEBUG_LIDAR_TF)
      {
        _model.state.debug[2] = dx;
        _model.state.debug[3] = dy;
      }
      updated = 1;
    }
  }
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
  if (_model.state.aux.color.present && now - _lastColorMs >= 700)
  {
    _lastColorMs = now;
    Device::ColorTCS34725Data color;
    if (_color.read(color))
    {
      _model.state.aux.color.clear = color.clear;
      _model.state.aux.color.red = color.red;
      _model.state.aux.color.green = color.green;
      _model.state.aux.color.blue = color.blue;
      _model.state.aux.color.lastUpdate = now;
      if (_model.config.debug.mode == DEBUG_ADC_INTERNAL)
      {
        _model.state.debug[0] = debugClamp(color.red);
        _model.state.debug[1] = debugClamp(color.green);
        _model.state.debug[2] = debugClamp(color.blue);
        _model.state.debug[3] = debugClamp(color.clear);
      }
      updated = 1;
    }
  }
#endif

  return updated;
#else
  return 0;
#endif
}

} // namespace Espfc::Sensor
