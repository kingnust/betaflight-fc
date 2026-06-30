#pragma once

#include "Model.h"
#include "Control/Fusion.h"
#include "Control/Altitude.hpp"
#include "Sensor/GyroSensor.h"
#include "Sensor/AccelSensor.h"
#include "Sensor/MagSensor.hpp"
#include "Sensor/BaroSensor.hpp"
#include "Sensor/VoltageSensor.h"
#include "Sensor/AuxSensor.hpp"
#include "Device/Input/InputButton.hpp"

namespace Espfc {

class SensorManager
{
  public:
    SensorManager(Model& model);

    int begin();
    int read();
    int preLoop();
    int postLoop();
    int fusion();
    // main task
    int update();
    // sub task
    int updateDelayed();

  private:
    Model& _model;
    Sensor::GyroSensor _gyro;
    Sensor::AccelSensor _accel;
    Sensor::MagSensor _mag;
    Sensor::BaroSensor _baro;
    Sensor::VoltageSensor _voltage;
    Sensor::AuxSensor _aux;
    Control::Fusion _fusion;
    Control::Altitude _altitude;
    bool _fusionUpdate;
    Device::Input::InputButton _button;
};

}
