#include "Sensor/AuxSensor.hpp"

#include "Hardware.h"
#include <cmath>

#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
namespace {

TwoWire vl53Wire(1);

constexpr uint8_t VL53L1X_ADDRESS = 0x29;

int16_t debugClamp(uint32_t value)
{
  return value > 32767u ? 32767 : (int16_t)value;
}

bool i2cDevicePresent(TwoWire& bus, uint8_t address)
{
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

} // namespace
#endif

namespace Espfc::Sensor {

AuxSensor::AuxSensor(Model& model): _model(model) {}

int AuxSensor::begin()
{
#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
#if defined(ESPFC_SPI_0)
  if (Device::BusSPI* spi = Device::getMainSpiBus())
  {
    _model.state.aux.flow.present = _flow.begin(spi, ESPFC_PMW3901_CS);
    _model.logger.info().log(F("AUX PMW3901")).logln(_model.state.aux.flow.present ? "Y" : "");
  }
#endif

#if defined(ESPFC_I2C_0)
  if (Device::BusI2C* i2c = Device::getMainI2cBus())
  {
    _model.state.aux.color.present = _color.begin(i2c, ESPFC_TCS_LED_PIN);
    _model.logger.info().log(F("AUX TCS34725")).logln(_model.state.aux.color.present ? "Y" : "");
  }
#endif

  vl53Wire.begin(ESPFC_VL53_I2C_SDA, ESPFC_VL53_I2C_SCL, _model.config.i2cSpeed * 1000ul);
  vl53Wire.setTimeOut(50);
  delay(10);
  if (i2cDevicePresent(vl53Wire, VL53L1X_ADDRESS))
  {
    _range.setBus(&vl53Wire);
    _range.setTimeout(50);
    _rangeStarted = _range.init();
    if (_rangeStarted)
    {
      _range.setDistanceMode(VL53L1X::Long);
      _range.setMeasurementTimingBudget(50000);
      _range.startContinuous(50);
    }
  }
  _model.state.aux.range.present = _rangeStarted;
  _model.logger.info().log(F("AUX VL53L1X")).logln(_rangeStarted ? "Y" : "");
#endif

  return 1;
}

int AuxSensor::update()
{
#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
  const uint32_t now = millis();
  int updated = 0;

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

  if (_model.state.aux.range.present && now - _lastRangeMs >= 50)
  {
    _lastRangeMs = now;
    if (_range.dataReady())
    {
      const uint16_t distance = _range.read(false);
      _model.state.aux.range.distanceMm = distance;
      _model.state.aux.range.status = (uint8_t)_range.ranging_data.range_status;
      _model.state.aux.range.signal = debugClamp(lrintf(_range.ranging_data.peak_signal_count_rate_MCPS * 1000.0f));
      _model.state.aux.range.ambient = debugClamp(lrintf(_range.ranging_data.ambient_count_rate_MCPS * 1000.0f));
      _model.state.aux.range.lastUpdate = now;
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
      updated = 1;
    }
  }

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

  return updated;
#else
  return 0;
#endif
}

} // namespace Espfc::Sensor
