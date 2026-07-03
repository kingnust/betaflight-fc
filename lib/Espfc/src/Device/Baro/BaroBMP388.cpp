#include "BaroBMP388.hpp"
#include "Debug_Espfc.h"
#include <Arduino.h>
#include <cmath>

#define BMP388_ADDRESS_FIRST       0x77
#define BMP388_ADDRESS_SECOND      0x76

#define BMP388_CHIP_ID_REG         0x00
#define BMP388_STATUS_REG          0x03
#define BMP388_DATA_REG            0x04
#define BMP388_PWR_CTRL_REG        0x1B
#define BMP388_OSR_REG             0x1C
#define BMP388_ODR_REG             0x1D
#define BMP388_CONFIG_REG          0x1F
#define BMP388_CALIB_DATA_REG      0x31
#define BMP388_CMD_REG             0x7E

#define BMP388_CHIP_ID             0x50
#define BMP390_CHIP_ID             0x60
#define BMP388_SOFT_RESET          0xB6

#define BMP388_PRESS_EN            0x01
#define BMP388_TEMP_EN             0x02
#define BMP388_MODE_FORCED         0x10
#define BMP388_MODE_SLEEP          0x00
#define BMP388_DRDY_PRESS          0x20
#define BMP388_DRDY_TEMP           0x40

#define BMP388_PRESS_OS_4X         0x02
#define BMP388_TEMP_OS_8X          0x18
#define BMP388_ODR_50HZ            0x02
#define BMP388_IIR_COEFF_3         0x04

namespace Espfc::Device::Baro {

int BaroBMP388::begin(BusDevice* bus)
{
  return begin(bus, BMP388_ADDRESS_FIRST) ? 1 : begin(bus, BMP388_ADDRESS_SECOND) ? 1 : 0;
}

int BaroBMP388::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  DRONE_PROTO_DEBUG_VALUE("bmp388 begin addr", addr);
  if (_bus->isSPI()) return 0;
  if (!testConnection())
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 first test failed");
    return 0;
  }

  if (!writeReg(BMP388_CMD_REG, BMP388_SOFT_RESET))
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 reset write failed");
    return 0;
  }
  delay(10);

  if (!testConnection())
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 second test failed");
    return 0;
  }
  if (!readCalibration())
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 calibration read failed");
    return 0;
  }

  if (!writeReg(BMP388_PWR_CTRL_REG, BMP388_MODE_SLEEP))
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 standby failed");
    return 0;
  }
  delay(5);
  if (!writeReg(BMP388_OSR_REG, BMP388_TEMP_OS_8X | BMP388_PRESS_OS_4X))
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 osr failed");
    return 0;
  }
  if (!writeReg(BMP388_ODR_REG, BMP388_ODR_50HZ))
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 odr failed");
    return 0;
  }
  if (!writeReg(BMP388_CONFIG_REG, BMP388_IIR_COEFF_3))
  {
    DRONE_PROTO_DEBUG_LINE("bmp388 config failed");
    return 0;
  }
  if (!readMeasurement()) return 0;

  return 1;
}

BaroDeviceType BaroBMP388::getType() const
{
  return BARO_BMP388;
}

float BaroBMP388::readTemperature()
{
  return _temperature;
}

float BaroBMP388::readPressure()
{
  readMeasurement();
  return _pressure;
}

void BaroBMP388::setMode(BaroDeviceMode mode)
{
  (void)mode;
}

int BaroBMP388::getDelay(BaroDeviceMode mode) const
{
  switch (mode)
  {
    case BARO_MODE_TEMP: return 0;
    default: return 25000;
  }
}

bool BaroBMP388::testConnection()
{
  uint8_t chipId = 0;
  const bool readOk = _bus->read(_addr, BMP388_CHIP_ID_REG, 1, &chipId) == 1;
  DRONE_PROTO_DEBUG_VALUE("bmp388 addr", _addr);
  DRONE_PROTO_DEBUG_HEX("bmp388 chip_id", chipId);
  return readOk && (chipId == BMP388_CHIP_ID || chipId == BMP390_CHIP_ID);
}

bool BaroBMP388::writeReg(uint8_t reg, uint8_t value)
{
  return _bus->write(_addr, reg, 1, &value);
}

bool BaroBMP388::readMeasurement()
{
  uint8_t data[6] = {0};
  if (!writeReg(BMP388_PWR_CTRL_REG, BMP388_MODE_FORCED | BMP388_TEMP_EN | BMP388_PRESS_EN)) return false;
  delay(30);

  uint8_t status = 0;
  if (_bus->read(_addr, BMP388_STATUS_REG, 1, &status) != 1) return false;
  if ((status & (BMP388_DRDY_PRESS | BMP388_DRDY_TEMP)) != (BMP388_DRDY_PRESS | BMP388_DRDY_TEMP))
  {
    DRONE_PROTO_DEBUG_HEX("bmp388 data not ready", status);
    return false;
  }

  if (_bus->readFast(_addr, BMP388_DATA_REG, 6, data) != 6) return false;

  uint32_t rawPressure = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
  uint32_t rawTemperature = ((uint32_t)data[5] << 16) | ((uint32_t)data[4] << 8) | data[3];
  if (rawPressure == 0 || rawTemperature == 0) return false;

  _temperature = (float)compensateTemperature(rawTemperature);
  _pressure = (float)compensatePressure(rawPressure);
  if (!std::isfinite(_temperature) || !std::isfinite(_pressure) || _pressure < 30000.0f || _pressure > 120000.0f)
  {
    DRONE_PROTO_DEBUG_VALUE("bmp388 bad pressure", _pressure);
    return false;
  }

  return true;
}

bool BaroBMP388::readCalibration()
{
  uint8_t data[21] = {0};
  if (_bus->read(_addr, BMP388_CALIB_DATA_REG, sizeof(data), data) != sizeof(data)) return false;
  parseCalibration(data);
  return true;
}

void BaroBMP388::parseCalibration(const uint8_t* data)
{
  auto u16 = [](uint8_t msb, uint8_t lsb) -> uint16_t {
    return ((uint16_t)msb << 8) | lsb;
  };

  uint16_t parT1 = u16(data[1], data[0]);
  uint16_t parT2 = u16(data[3], data[2]);
  int8_t parT3 = (int8_t)data[4];
  int16_t parP1 = (int16_t)u16(data[6], data[5]);
  int16_t parP2 = (int16_t)u16(data[8], data[7]);
  int8_t parP3 = (int8_t)data[9];
  int8_t parP4 = (int8_t)data[10];
  uint16_t parP5 = u16(data[12], data[11]);
  uint16_t parP6 = u16(data[14], data[13]);
  int8_t parP7 = (int8_t)data[15];
  int8_t parP8 = (int8_t)data[16];
  int16_t parP9 = (int16_t)u16(data[18], data[17]);
  int8_t parP10 = (int8_t)data[19];
  int8_t parP11 = (int8_t)data[20];

  _cal.par_t1 = (double)parT1 / 0.00390625;
  _cal.par_t2 = (double)parT2 / 1073741824.0;
  _cal.par_t3 = (double)parT3 / 281474976710656.0;
  _cal.par_p1 = (double)(parP1 - 16384) / 1048576.0;
  _cal.par_p2 = (double)(parP2 - 16384) / 536870912.0;
  _cal.par_p3 = (double)parP3 / 4294967296.0;
  _cal.par_p4 = (double)parP4 / 137438953472.0;
  _cal.par_p5 = (double)parP5 / 0.125;
  _cal.par_p6 = (double)parP6 / 64.0;
  _cal.par_p7 = (double)parP7 / 256.0;
  _cal.par_p8 = (double)parP8 / 32768.0;
  _cal.par_p9 = (double)parP9 / 281474976710656.0;
  _cal.par_p10 = (double)parP10 / 281474976710656.0;
  _cal.par_p11 = (double)parP11 / 36893488147419103232.0;
}

double BaroBMP388::compensateTemperature(uint32_t rawTemperature)
{
  double partial1 = (double)rawTemperature - _cal.par_t1;
  double partial2 = partial1 * _cal.par_t2;
  _cal.t_lin = partial2 + partial1 * partial1 * _cal.par_t3;
  return _cal.t_lin;
}

double BaroBMP388::compensatePressure(uint32_t rawPressure) const
{
  double t = _cal.t_lin;
  double t2 = t * t;
  double t3 = t2 * t;
  double p = (double)rawPressure;
  double p2 = p * p;
  double p3 = p2 * p;

  double partialOut1 = _cal.par_p5 + _cal.par_p6 * t + _cal.par_p7 * t2 + _cal.par_p8 * t3;
  double partialOut2 = p * (_cal.par_p1 + _cal.par_p2 * t + _cal.par_p3 * t2 + _cal.par_p4 * t3);
  double partialData = p2 * (_cal.par_p9 + _cal.par_p10 * t) + p3 * _cal.par_p11;

  return partialOut1 + partialOut2 + partialData;
}

} // namespace Espfc::Device::Baro
