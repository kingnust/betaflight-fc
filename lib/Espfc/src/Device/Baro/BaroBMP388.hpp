#pragma once

#include "Debug_Espfc.h"
#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroBMP388 : public BaroDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  BaroDeviceType getType() const final;

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode) final;
  int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

private:
  struct CalibrationData
  {
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
  };

  bool writeReg(uint8_t reg, uint8_t value);
  bool readMeasurement();
  bool readCalibration();
  void parseCalibration(const uint8_t* data);
  double compensateTemperature(uint32_t rawTemperature);
  double compensatePressure(uint32_t rawPressure) const;

  CalibrationData _cal;
  float _temperature = 0.0f;
  float _pressure = 0.0f;
};

} // namespace Espfc::Device::Baro

