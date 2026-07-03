#include "BaroSensor.hpp"
#include "Utils/Math.hpp"
#include <cmath>
#include <functional>

namespace Espfc::Sensor {

BaroSensor::BaroSensor(Model& model): _model(model), _state(BARO_STATE_INIT), _counter(0), _pressureReady(false) {}

int BaroSensor::begin()
{
  if (!_model.baroActive() || !_model.state.baro.dev) return 0;

  _baro = _model.state.baro.dev;

  const int gyroInterval = _model.state.gyro.timer.interval > 0 ? _model.state.gyro.timer.interval : 1000;
  const int delay = _baro->getDelay(BARO_MODE_TEMP) + _baro->getDelay(BARO_MODE_PRESS);
  const int toGyroRate = (delay / gyroInterval) + 1; // number of gyro readings per cycle
  const int interval = gyroInterval * toGyroRate;
  const int rate = 1000000 / interval;
  const int biasSamples = 3 * rate;
  const auto internalFilter = FILTER_PT1;
  const auto internalCutoff = std::max((rate + 4) / 8, 1);

  _temperatureFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
  _pressureFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
  _altitudeFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
  _varioFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);

  _temperatureMedianFilter.begin(FilterConfig(FILTER_MEDIAN3, 0), rate);
  _pressureMedianFilter.begin(FilterConfig(FILTER_MEDIAN3, 0), rate);

  _model.logger.info()
      .log(F("BARO INIT"))
      .log(FPSTR(Device::BaroDevice::getName(_baro->getType())))
      .log(rate)
      .logln(internalCutoff);

  _model.state.baro.rate = rate;
  _model.state.baro.altitudeBiasSamples = biasSamples;
  _baro->setMode(BARO_MODE_TEMP);

  return 1;
}

int BaroSensor::update()
{
  int status = read();

  return status;
}

int BaroSensor::read()
{
  if (!_baro || !_model.baroActive()) return 0;

  if (_wait > micros()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BARO);

  // if(_model.config.debug.mode == DEBUG_BARO)
  // {
  //   _model.state.debug[0] = _state;
  // }

  switch (_state)
  {
    case BARO_STATE_INIT:
      _baro->setMode(BARO_MODE_TEMP);
      _state = BARO_STATE_TEMP_GET;
      _wait = micros() + _baro->getDelay(BARO_MODE_TEMP);
      return 0;
    case BARO_STATE_TEMP_GET:
      readTemperature();
      _baro->setMode(BARO_MODE_PRESS);
      _state = BARO_STATE_PRESS_GET;
      _wait = micros() + _baro->getDelay(BARO_MODE_PRESS);
      _counter = 1;
      return 1;
    case BARO_STATE_PRESS_GET:
      if (readPressure())
      {
        updateAltitude();
      }
      if (--_counter > 0)
      {
        _baro->setMode(BARO_MODE_PRESS);
        _state = BARO_STATE_PRESS_GET;
        _wait = micros() + _baro->getDelay(BARO_MODE_PRESS);
      }
      else
      {
        _baro->setMode(BARO_MODE_TEMP);
        _state = BARO_STATE_TEMP_GET;
        _wait = micros() + _baro->getDelay(BARO_MODE_TEMP);
      }
      return 1;
      break;
    default: _state = BARO_STATE_INIT; break;
  }

  return 0;
}

void BaroSensor::readTemperature()
{
  float temp = _model.state.baro.temperatureRaw = _baro->readTemperature();
  // temp = _temperatureMedianFilter.update(temp);
  _model.state.baro.temperature = _temperatureFilter.update(temp);
}

bool BaroSensor::readPressure()
{
  float press = _model.state.baro.pressureRaw = _baro->readPressure();
  if (!std::isfinite(press) || press < 30000.0f || press > 120000.0f)
  {
    return false;
  }

  if (!_pressureReady)
  {
    for (int i = 0; i < 8; i++)
    {
      _model.state.baro.pressure = _pressureFilter.update(press);
    }
    _model.state.baro.altitudeRaw = Utils::toAltitude(_model.state.baro.pressure);
    for (int i = 0; i < 8; i++)
    {
      _model.state.baro.altitude = _altitudeFilter.update(_model.state.baro.altitudeRaw);
    }
    _model.state.baro.altitudeBias = _model.state.baro.altitude;
    _model.state.baro.altitudeGround = 0.0f;
    _model.state.baro.altitudePrev = _model.state.baro.altitude;
    _model.state.baro.vario = 0.0f;
    _pressureReady = true;
    return false;
  }

  // press = _pressureMedianFilter.update(press);
  _model.state.baro.pressure = _pressureFilter.update(press);
  return true;
}

void BaroSensor::updateAltitude()
{
  Espfc::BaroState& baro = _model.state.baro;

  baro.altitudeRaw = Utils::toAltitude(baro.pressure);
  baro.altitude = _altitudeFilter.update(baro.altitudeRaw);

  if (baro.altitudeBiasSamples > 0)
  {
    baro.altitudeBiasSamples--;
    baro.altitudeBias += (baro.altitude - baro.altitudeBias) * (5.0f / baro.rate);
  }
  else if (baro.altitudeBiasSamples == 0)
  {
    _model.logger.info().log("BARO BIAS").logln(baro.altitudeBias);
    baro.altitudeBiasSamples--;
  }

  baro.altitudeGround = baro.altitude - baro.altitudeBias;

  const float varioAlt = baro.altitude;
  baro.vario = _varioFilter.update((varioAlt - baro.altitudePrev) * baro.rate);
  baro.altitudePrev = varioAlt;

  if (_model.config.debug.mode == DEBUG_BARO)
  {
    _model.state.debug[0] = lrintf(baro.vario * 100.0f);     // cm/s
    _model.state.debug[1] = lrintf(baro.pressureRaw * 0.1f); // hPa x 10
    //_model.state.debug[1] = lrintf(baro.pressureRaw - 100000.0f); // Pa - 100000
    _model.state.debug[2] = lrintf(baro.temperatureRaw * 100.f); // deg C x 100
    _model.state.debug[3] = lrintf(baro.altitudeGround * 100.f); // cm
  }
}

} // namespace Espfc::Sensor
