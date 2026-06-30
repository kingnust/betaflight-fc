#include "MagBMM150.hpp"
#include <Arduino.h>

#define BMM150_ADDRESS_1                 0x10
#define BMM150_ADDRESS_2                 0x11
#define BMM150_ADDRESS_3                 0x12
#define BMM150_ADDRESS_4                 0x13

#define BMM150_CHIP_ID_REG               0x40
#define BMM150_CHIP_ID                   0x32
#define BMM150_DATA_X_LSB_REG            0x42
#define BMM150_POWER_CONTROL_REG         0x4B
#define BMM150_OP_MODE_REG               0x4C
#define BMM150_AXES_ENABLE_REG           0x4E
#define BMM150_REP_XY_REG                0x51
#define BMM150_REP_Z_REG                 0x52

#define BMM150_DIG_X1                    0x5D
#define BMM150_DIG_Y1                    0x5E
#define BMM150_DIG_Z4_LSB                0x62
#define BMM150_DIG_Z2_LSB                0x68

#define BMM150_POWER_ENABLE              0x01
#define BMM150_POWER_DISABLE             0x00
#define BMM150_MODE_NORMAL               0x00
#define BMM150_DATA_RATE_30HZ            0x07
#define BMM150_REPXY_HIGHACCURACY        0x17
#define BMM150_REPZ_HIGHACCURACY         0x29
#define BMM150_XYZ_CHANNEL_ENABLE        0x00

#define BMM150_DATA_X_MSK                0xF8
#define BMM150_DATA_X_POS                0x03
#define BMM150_DATA_Y_MSK                0xF8
#define BMM150_DATA_Y_POS                0x03
#define BMM150_DATA_Z_MSK                0xFE
#define BMM150_DATA_Z_POS                0x01
#define BMM150_DATA_RHALL_MSK            0xFC
#define BMM150_DATA_RHALL_POS            0x02

#define BMM150_OVERFLOW_ADCVAL_XYAXES    -4096
#define BMM150_OVERFLOW_ADCVAL_ZAXIS     -16384
#define BMM150_OVERFLOW_OUTPUT           -32768
#define BMM150_NEGATIVE_SATURATION_Z     -32767
#define BMM150_POSITIVE_SATURATION_Z     32767

namespace Espfc::Device::Mag {

int MagBMM150::begin(BusDevice* bus)
{
  return begin(bus, BMM150_ADDRESS_1) ? 1 :
         begin(bus, BMM150_ADDRESS_2) ? 1 :
         begin(bus, BMM150_ADDRESS_3) ? 1 :
         begin(bus, BMM150_ADDRESS_4) ? 1 : 0;
}

int MagBMM150::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (_bus->isSPI()) return 0;
  if (!testConnection()) return 0;
  if (!readTrimData()) return 0;

  setOperationMode(BMM150_MODE_NORMAL);
  setHighAccuracyPreset();
  setRate30Hz();
  setMeasurementXYZ();
  delay(10);

  return 1;
}

int MagBMM150::readMag(VectorInt16& v)
{
  uint8_t data[8] = {0};
  if (_bus->readFast(_addr, BMM150_DATA_X_LSB_REG, sizeof(data), data) != sizeof(data)) return 0;

  int16_t rawX = (int16_t)(((int16_t)((int8_t)data[1]) * 32) | ((data[0] & BMM150_DATA_X_MSK) >> BMM150_DATA_X_POS));
  int16_t rawY = (int16_t)(((int16_t)((int8_t)data[3]) * 32) | ((data[2] & BMM150_DATA_Y_MSK) >> BMM150_DATA_Y_POS));
  int16_t rawZ = (int16_t)(((int16_t)((int8_t)data[5]) * 128) | ((data[4] & BMM150_DATA_Z_MSK) >> BMM150_DATA_Z_POS));
  uint16_t rhall = (uint16_t)(((uint16_t)data[7] << 6) | ((data[6] & BMM150_DATA_RHALL_MSK) >> BMM150_DATA_RHALL_POS));

  v.x = compensateX(rawX, rhall);
  v.y = compensateY(rawY, rhall);
  v.z = compensateZ(rawZ, rhall);

  return 1;
}

const VectorFloat MagBMM150::convert(const VectorInt16& v) const
{
  return VectorFloat{v};
}

int MagBMM150::getRate() const
{
  return 30;
}

MagDeviceType MagBMM150::getType() const
{
  return MAG_BMM150;
}

bool MagBMM150::testConnection()
{
  setPower(true);
  delay(3);

  uint8_t chipId = 0;
  return _bus->read(_addr, BMM150_CHIP_ID_REG, 1, &chipId) == 1 && chipId == BMM150_CHIP_ID;
}

bool MagBMM150::writeReg(uint8_t reg, uint8_t value)
{
  delay(3);
  return _bus->write(_addr, reg, 1, &value);
}

bool MagBMM150::readTrimData()
{
  uint8_t trimX1Y1[2] = {0};
  uint8_t trimXYXData[4] = {0};
  uint8_t trimXY1XY2[10] = {0};

  if (_bus->read(_addr, BMM150_DIG_X1, sizeof(trimX1Y1), trimX1Y1) != sizeof(trimX1Y1)) return false;
  if (_bus->read(_addr, BMM150_DIG_Z4_LSB, sizeof(trimXYXData), trimXYXData) != sizeof(trimXYXData)) return false;
  if (_bus->read(_addr, BMM150_DIG_Z2_LSB, sizeof(trimXY1XY2), trimXY1XY2) != sizeof(trimXY1XY2)) return false;

  _trim.digX1 = (int8_t)trimX1Y1[0];
  _trim.digY1 = (int8_t)trimX1Y1[1];
  _trim.digX2 = (int8_t)trimXYXData[2];
  _trim.digY2 = (int8_t)trimXYXData[3];
  _trim.digZ1 = (uint16_t)(((uint16_t)trimXY1XY2[3] << 8) | trimXY1XY2[2]);
  _trim.digZ2 = (int16_t)(((uint16_t)trimXY1XY2[1] << 8) | trimXY1XY2[0]);
  _trim.digZ3 = (int16_t)(((uint16_t)trimXY1XY2[7] << 8) | trimXY1XY2[6]);
  _trim.digZ4 = (int16_t)(((uint16_t)trimXYXData[1] << 8) | trimXYXData[0]);
  _trim.digXY1 = trimXY1XY2[9];
  _trim.digXY2 = (int8_t)trimXY1XY2[8];
  _trim.digXYZ1 = (uint16_t)(((uint16_t)(trimXY1XY2[5] & 0x7F) << 8) | trimXY1XY2[4]);

  return true;
}

void MagBMM150::setPower(bool enabled)
{
  writeReg(BMM150_POWER_CONTROL_REG, enabled ? BMM150_POWER_ENABLE : BMM150_POWER_DISABLE);
}

void MagBMM150::setOperationMode(uint8_t mode)
{
  setPower(true);
  uint8_t reg = 0;
  _bus->read(_addr, BMM150_OP_MODE_REG, 1, &reg);
  reg = (reg & ~0x06) | ((mode << 1) & 0x06);
  writeReg(BMM150_OP_MODE_REG, reg);
}

void MagBMM150::setRate30Hz()
{
  uint8_t reg = 0;
  _bus->read(_addr, BMM150_OP_MODE_REG, 1, &reg);
  reg = (reg & ~0x38) | ((BMM150_DATA_RATE_30HZ << 3) & 0x38);
  writeReg(BMM150_OP_MODE_REG, reg);
}

void MagBMM150::setHighAccuracyPreset()
{
  writeReg(BMM150_REP_XY_REG, BMM150_REPXY_HIGHACCURACY);
  writeReg(BMM150_REP_Z_REG, BMM150_REPZ_HIGHACCURACY);
}

void MagBMM150::setMeasurementXYZ()
{
  uint8_t reg = 0;
  _bus->read(_addr, BMM150_AXES_ENABLE_REG, 1, &reg);
  reg = (reg & ~0x38) | BMM150_XYZ_CHANNEL_ENABLE;
  writeReg(BMM150_AXES_ENABLE_REG, reg);
}

int16_t MagBMM150::compensateX(int16_t rawX, uint16_t rhall) const
{
  if (rawX == BMM150_OVERFLOW_ADCVAL_XYAXES) return BMM150_OVERFLOW_OUTPUT;

  uint16_t processCompX0 = rhall != 0 ? rhall : _trim.digXYZ1;
  if (processCompX0 == 0) return BMM150_OVERFLOW_OUTPUT;

  int16_t processCompX1 = ((int32_t)_trim.digXYZ1) * 16384;
  uint16_t processCompX2 = ((uint16_t)(processCompX1 / processCompX0)) - ((uint16_t)0x4000);
  int16_t retval = ((int16_t)processCompX2);
  int32_t processCompX3 = ((int32_t)retval) * ((int32_t)retval);
  int32_t processCompX4 = ((int32_t)_trim.digXY2) * (processCompX3 / 128);
  int32_t processCompX5 = ((int32_t)((int16_t)_trim.digXY1)) * 128;
  int32_t processCompX6 = ((int32_t)retval) * processCompX5;
  int32_t processCompX7 = (((processCompX4 + processCompX6) / 512) + ((int32_t)0x100000));
  int32_t processCompX8 = ((int32_t)(((int16_t)_trim.digX2) + ((int16_t)0xA0)));
  int32_t processCompX9 = ((processCompX7 * processCompX8) / 4096);
  int32_t processCompX10 = ((int32_t)rawX) * processCompX9;
  retval = ((int16_t)(processCompX10 / 8192));
  retval = (retval + (((int16_t)_trim.digX1) * 8)) / 16;
  return retval;
}

int16_t MagBMM150::compensateY(int16_t rawY, uint16_t rhall) const
{
  if (rawY == BMM150_OVERFLOW_ADCVAL_XYAXES) return BMM150_OVERFLOW_OUTPUT;

  uint16_t processCompY0 = rhall != 0 ? rhall : _trim.digXYZ1;
  if (processCompY0 == 0) return BMM150_OVERFLOW_OUTPUT;

  int32_t processCompY1 = (((int32_t)_trim.digXYZ1) * 16384) / processCompY0;
  uint16_t processCompY2 = ((uint16_t)processCompY1) - ((uint16_t)0x4000);
  int16_t retval = ((int16_t)processCompY2);
  int32_t processCompY3 = ((int32_t)retval) * ((int32_t)retval);
  int32_t processCompY4 = ((int32_t)_trim.digXY2) * (processCompY3 / 128);
  int32_t processCompY5 = ((int32_t)(((int16_t)_trim.digXY1) * 128));
  int32_t processCompY6 = ((processCompY4 + (((int32_t)retval) * processCompY5)) / 512);
  int32_t processCompY7 = ((int32_t)(((int16_t)_trim.digY2) + ((int16_t)0xA0)));
  int32_t processCompY8 = (((processCompY6 + ((int32_t)0x100000)) * processCompY7) / 4096);
  int32_t processCompY9 = (((int32_t)rawY) * processCompY8);
  retval = (int16_t)(processCompY9 / 8192);
  retval = (retval + (((int16_t)_trim.digY1) * 8)) / 16;
  return retval;
}

int16_t MagBMM150::compensateZ(int16_t rawZ, uint16_t rhall) const
{
  if (rawZ == BMM150_OVERFLOW_ADCVAL_ZAXIS) return BMM150_OVERFLOW_OUTPUT;

  if (_trim.digZ2 == 0 || _trim.digZ1 == 0 || rhall == 0 || _trim.digXYZ1 == 0) return BMM150_OVERFLOW_OUTPUT;

  int16_t processCompZ0 = ((int16_t)rhall) - ((int16_t)_trim.digXYZ1);
  int32_t processCompZ1 = (((int32_t)_trim.digZ3) * ((int32_t)processCompZ0)) / 4;
  int32_t processCompZ2 = (((int32_t)(rawZ - _trim.digZ4)) * 32768);
  int32_t processCompZ3 = ((int32_t)_trim.digZ1) * (((int16_t)rhall) * 2);
  int16_t processCompZ4 = (int16_t)((processCompZ3 + 32768) / 65536);
  int32_t retval = ((processCompZ2 - processCompZ1) / (_trim.digZ2 + processCompZ4));

  if (retval > BMM150_POSITIVE_SATURATION_Z) retval = BMM150_POSITIVE_SATURATION_Z;
  else if (retval < BMM150_NEGATIVE_SATURATION_Z) retval = BMM150_NEGATIVE_SATURATION_Z;

  return (int16_t)(retval / 16);
}

} // namespace Espfc::Device::Mag

