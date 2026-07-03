#include "Sensor/AuxSensor.hpp"

#include "Hardware.h"
#include <cmath>

#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
namespace {

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
TwoWire vl53Wire(1);

constexpr uint8_t VL53L1X_ADDRESS = 0x29;
constexpr uint16_t VL53L1X_MODEL_ID = 0xEACC;
constexpr uint32_t VL53L1X_I2C_SPEED = 100000;
constexpr uint32_t VL53L1X_INIT_DELAY_MS = 1000;
constexpr uint32_t VL53L1X_INIT_RETRY_MS = 2000;
constexpr uint32_t VL53L1X_UPDATE_MS = 100;
constexpr uint8_t VL53L1X_I2C_TIMEOUT_MS = 5;
constexpr uint8_t VL53L1X_SENSOR_TIMEOUT_MS = 10;
constexpr uint8_t VL53L1X_MAX_FAILURES = 5;
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X) || defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
int16_t debugClamp(uint32_t value)
{
  return value > 32767u ? 32767 : (int16_t)value;
}
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
bool i2cDevicePresent(TwoWire& bus, uint8_t address)
{
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

bool i2cReadReg16(TwoWire& bus, uint8_t address, uint16_t reg, uint8_t* data, uint8_t len)
{
  bus.beginTransmission(address);
  bus.write((uint8_t)(reg >> 8));
  bus.write((uint8_t)reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(address, len) != len) return false;
  for (uint8_t i = 0; i < len; i++)
  {
    data[i] = bus.read();
  }
  return true;
}
#endif

} // namespace
#endif

namespace Espfc::Sensor {

AuxSensor::AuxSensor(Model& model): _model(model) {}

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
void AuxSensor::beginRangefinder(uint32_t now)
{
  _lastRangeInitMs = now;
  _rangeFailures = 0;
  _rangeStarted = false;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = 255;

  if (!_rangeBusStarted)
  {
    pinMode(ESPFC_VL53_I2C_SDA, INPUT_PULLUP);
    pinMode(ESPFC_VL53_I2C_SCL, INPUT_PULLUP);
    vl53Wire.begin(ESPFC_VL53_I2C_SDA, ESPFC_VL53_I2C_SCL, VL53L1X_I2C_SPEED);
    vl53Wire.setTimeOut(VL53L1X_I2C_TIMEOUT_MS);
    _rangeBusStarted = true;
    delay(2);
  }

  if (!i2cDevicePresent(vl53Wire, VL53L1X_ADDRESS))
  {
    _model.state.aux.range.status = 252;
    return;
  }

  uint8_t id[2] = {0, 0};
  if (!i2cReadReg16(vl53Wire, VL53L1X_ADDRESS, VL53L1X::IDENTIFICATION__MODEL_ID, id, sizeof(id)))
  {
    _model.state.aux.range.status = 251;
    return;
  }

  const uint16_t modelId = ((uint16_t)id[0] << 8) | id[1];
  if (modelId != VL53L1X_MODEL_ID)
  {
    _model.state.aux.range.status = 250;
    return;
  }

  _range.setBus(&vl53Wire);
  _range.setTimeout(VL53L1X_SENSOR_TIMEOUT_MS);
  if (!_range.init())
  {
    _model.state.aux.range.status = _range.timeoutOccurred() ? 248 : 249;
    return;
  }

  if (!_range.setDistanceMode(VL53L1X::Long))
  {
    _model.state.aux.range.status = 247;
    return;
  }

  if (!_range.setMeasurementTimingBudget(50000))
  {
    _model.state.aux.range.status = 246;
    return;
  }

  _range.startContinuous(VL53L1X_UPDATE_MS);
  if (_range.timeoutOccurred() || _range.last_status != 0)
  {
    _model.state.aux.range.status = 245;
    return;
  }

  _rangeStarted = true;
  _model.state.aux.range.present = true;
  _model.state.aux.range.status = 254;
  _model.logger.info().log(F("AUX VL53L1X")).logln(F("Y"));
}

bool AuxSensor::rangefinderFailure(uint8_t status, uint32_t now)
{
  _model.state.aux.range.status = status;
  _rangeFailures++;
  if (_rangeFailures >= VL53L1X_MAX_FAILURES)
  {
    _rangeStarted = false;
    _model.state.aux.range.present = false;
    _lastRangeInitMs = now;
  }
  return false;
}

void AuxSensor::rangefinderTaskEntry(void* arg)
{
  static_cast<AuxSensor*>(arg)->rangefinderTask();
  vTaskDelete(nullptr);
}

void AuxSensor::rangefinderTask()
{
  delay(VL53L1X_INIT_DELAY_MS);
  _lastRangeInitMs = 0;

  for (;;)
  {
    const uint32_t now = millis();

    if (!_rangeStarted)
    {
      if (_lastRangeInitMs == 0 || now - _lastRangeInitMs >= VL53L1X_INIT_RETRY_MS)
      {
        beginRangefinder(now);
      }
      vTaskDelay(pdMS_TO_TICKS(_rangeStarted ? VL53L1X_UPDATE_MS : VL53L1X_INIT_RETRY_MS));
      continue;
    }

    updateRangefinder(now);
    vTaskDelay(pdMS_TO_TICKS(VL53L1X_UPDATE_MS));
  }
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
    if (_range.timeoutOccurred() || _range.last_status != 0)
    {
      return rangefinderFailure(243, now);
    }
    if ((_model.state.aux.range.lastUpdate == 0 && now - _lastRangeInitMs > 1000) ||
        (_model.state.aux.range.lastUpdate != 0 && now - _model.state.aux.range.lastUpdate > 1000))
    {
      return rangefinderFailure(244, now);
    }
    return false;
  }

  const uint16_t distance = _range.read(false);
  if (_range.timeoutOccurred() || _range.last_status != 0)
  {
    return rangefinderFailure(242, now);
  }

  _model.state.aux.range.distanceMm = distance;
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
#endif

int AuxSensor::begin()
{
#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901) && defined(ESPFC_SPI_0)
  if (Device::BusSPI* spi = Device::getMainSpiBus())
  {
    _model.state.aux.flow.present = _flow.begin(spi, ESPFC_PMW3901_CS);
    _model.logger.info().log(F("AUX PMW3901")).logln(_model.state.aux.flow.present ? "Y" : "");
  }
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725) && defined(ESPFC_I2C_0)
  if (Device::BusI2C* i2c = Device::getMainI2cBus())
  {
    _model.state.aux.color.present = _color.begin(i2c, ESPFC_TCS_LED_PIN);
    _model.logger.info().log(F("AUX TCS34725")).logln(_model.state.aux.color.present ? "Y" : "");
  }
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
  _lastRangeInitMs = millis();
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = 255;
  if (!_rangeTask)
  {
    const BaseType_t started = xTaskCreate(rangefinderTaskEntry, "vl53Task", 6144, this, 0, &_rangeTask);
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
