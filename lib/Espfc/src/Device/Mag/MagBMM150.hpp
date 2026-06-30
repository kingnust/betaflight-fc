#pragma once

#include "Device/BusDevice.hpp"
#include "Device/MagDevice.hpp"

namespace Espfc::Device::Mag {

class MagBMM150 : public MagDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  int readMag(VectorInt16& v) final;
  const VectorFloat convert(const VectorInt16& v) const final;
  int getRate() const final;
  MagDeviceType getType() const final;
  bool testConnection() final;

private:
  struct TrimData
  {
    int8_t digX1;
    int8_t digY1;
    int8_t digX2;
    int8_t digY2;
    uint16_t digZ1;
    int16_t digZ2;
    int16_t digZ3;
    int16_t digZ4;
    uint8_t digXY1;
    int8_t digXY2;
    uint16_t digXYZ1;
  };

  bool writeReg(uint8_t reg, uint8_t value);
  bool readTrimData();
  void setPower(bool enabled);
  void setOperationMode(uint8_t mode);
  void setRate30Hz();
  void setHighAccuracyPreset();
  void setMeasurementXYZ();
  int16_t compensateX(int16_t rawX, uint16_t rhall) const;
  int16_t compensateY(int16_t rawY, uint16_t rhall) const;
  int16_t compensateZ(int16_t rawZ, uint16_t rhall) const;

  TrimData _trim;
};

} // namespace Espfc::Device::Mag

