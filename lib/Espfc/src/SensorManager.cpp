#include "SensorManager.h"
#include "Debug_Espfc.h"

namespace Espfc {

SensorManager::SensorManager(Model& model):
  _model(model), _gyro(model), _accel(model), _mag(model), _baro(model),
  _voltage(model), _aux(model), _fusion(model), _altitude(model), _fusionUpdate(false)
{
}

int SensorManager::begin()
{
  DRONE_PROTO_DEBUG_LINE("sensor.begin start");
  DRONE_PROTO_DEBUG_LINE("sensor before gyro.begin");
  _gyro.begin();
  DRONE_PROTO_DEBUG_LINE("sensor after gyro.begin");
#if defined(ESPFC_DRONE_PROTO_GYRO_ONLY) && defined(ESPFC_DRONE_PROTO_GYRO_NO_ACCEL)
  DRONE_PROTO_DEBUG_LINE("sensor gyro-only return");
  return 1;
#endif
  DRONE_PROTO_DEBUG_LINE("sensor before accel.begin");
  _accel.begin();
  DRONE_PROTO_DEBUG_LINE("sensor before mag.begin");
  _mag.begin();
  DRONE_PROTO_DEBUG_LINE("sensor before baro.begin");
  _baro.begin();
  DRONE_PROTO_DEBUG_LINE("sensor before voltage.begin");
  _voltage.begin();
  DRONE_PROTO_DEBUG_LINE("sensor before aux.begin");
  _aux.begin();
  DRONE_PROTO_DEBUG_LINE("sensor before fusion.begin");
  _fusion.begin();
  DRONE_PROTO_DEBUG_LINE("sensor before altitude.begin");
  _altitude.begin();
  DRONE_PROTO_DEBUG_LINE("sensor before button.begin");
  _button.begin(_model.config.pin[PIN_BUTTON]);
  DRONE_PROTO_DEBUG_LINE("sensor.begin done");

  return 1;
}

int FAST_CODE_ATTR SensorManager::read()
{
  _gyro.read();

  if(_model.state.loopTimer.syncTo(_model.state.gyro.timer))
  {
    _model.state.appQueue.send(Event(EVENT_GYRO_READ));
  }

  if(_model.state.accel.timer.syncTo(_model.state.gyro.timer))
  {
    _accel.update();
    _model.state.appQueue.send(Event(EVENT_ACCEL_READ));
    _model.state.mode.button = _button.update();
    return 1;
  }

  if(_mag.update()) return 1;

  if(_baro.update()) return 1;

  if(_voltage.update()) return 1;

  if(_aux.update()) return 1;

  return 0;
}

int FAST_CODE_ATTR SensorManager::preLoop()
{
  _gyro.filter();
  if(_model.state.gyro.biasSamples == 0)
  {
    _model.state.gyro.biasSamples = -1;
    _fusion.restoreGain();
  }
  return 1;
}

int SensorManager::postLoop()
{
  _gyro.postLoop();
  return 1;
}

int FAST_CODE_ATTR SensorManager::fusion()
{
  _fusion.update();
  _altitude.update();
  return 1;
}

// main task
int FAST_CODE_ATTR SensorManager::update()
{
  _gyro.read();
  return preLoop();
}

// sub task
int SensorManager::updateDelayed()
{
  _gyro.postLoop();

  // update at most one sensor besides gyro
  int status = 0;
  if(_model.state.accel.timer.syncTo(_model.state.gyro.timer))
  {
    _accel.update();
    _model.state.mode.button = _button.update();
    status = 1;
  }

  // delay imu update to next cycle
  if(_fusionUpdate)
  {
    _fusionUpdate = false;
    fusion();
  }
  _fusionUpdate = status;

  if(status) return 1;

  if(_mag.update()) return 1;

  if(_baro.update()) return 1;

  if(_voltage.update()) return 0;

  if(_aux.update()) return 0;

  return 0;
}

}
