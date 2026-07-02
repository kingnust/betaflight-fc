#pragma once

#include "Debug_Espfc.h"
#include "GyroDevice.h"
#include "Utils/MemoryHelper.h"
#include "helper_3dmath.h"
#include <Arduino.h>

#define BMI088_ACCEL_CHIP_ID_REG       0x00
#define BMI088_ACCEL_CHIP_ID           0x1E
#define BMI088_ACCEL_DATA_REG          0x12
#define BMI088_ACCEL_ODR_REG           0x40
#define BMI088_ACCEL_RANGE_REG         0x41
#define BMI088_ACCEL_PWR_CONF_REG      0x7C
#define BMI088_ACCEL_PWR_CTRL_REG      0x7D
#define BMI088_ACCEL_SOFT_RESET_REG    0x7E

#define BMI088_ACCEL_RANGE_12G         0x02
#define BMI088_ACCEL_ODR_1600_BW_280   0xAC
#define BMI088_ACCEL_POWER_ENABLE      0x04
#define BMI088_ACCEL_MODE_ACTIVE       0x00
#define BMI088_ACCEL_SOFT_RESET        0xB6

#define BMI088_GYRO_CHIP_ID_REG        0x00
#define BMI088_GYRO_CHIP_ID            0x0F
#define BMI088_GYRO_DATA_REG           0x02
#define BMI088_GYRO_RANGE_REG          0x0F
#define BMI088_GYRO_ODR_REG            0x10
#define BMI088_GYRO_SOFT_RESET_REG     0x14
#define BMI088_GYRO_INT_CTRL_REG       0x15

#define BMI088_GYRO_RANGE_2000DPS      0x00
#define BMI088_GYRO_ODR_2000_BW_230    0x81
#define BMI088_GYRO_SOFT_RESET         0xB6
#define BMI088_GYRO_DRDY_ENABLE        0x80

namespace Espfc::Device {

class GyroBMI088 : public GyroDevice
{
public:
  GyroBMI088(): _gyroAddr(0xFF) {}

  void setGyroCs(uint8_t addr)
  {
    _gyroAddr = addr;
  }

  int begin(BusDevice* bus) override
  {
    return begin(bus, 0);
  }

  int begin(BusDevice* bus, uint8_t addr) override
  {
    setBus(bus, addr);
    if (_gyroAddr == 0xFF) return 0;

    if (!testConnection()) return 0;

    if (!writeAccelByte(BMI088_ACCEL_SOFT_RESET_REG, BMI088_ACCEL_SOFT_RESET)) return 0;
    delay(50);
    if (!writeGyroByte(BMI088_GYRO_SOFT_RESET_REG, BMI088_GYRO_SOFT_RESET)) return 0;
    delay(50);

    if (!writeAccelByte(BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_POWER_ENABLE)) return 0;
    delay(5);
    if (!writeAccelByte(BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_MODE_ACTIVE)) return 0;
    delay(5);
    if (!writeAccelByte(BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_12G)) return 0;
    if (!writeAccelByte(BMI088_ACCEL_ODR_REG, BMI088_ACCEL_ODR_1600_BW_280)) return 0;

    if (!writeGyroByte(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS)) return 0;
    if (!writeGyroByte(BMI088_GYRO_ODR_REG, BMI088_GYRO_ODR_2000_BW_230)) return 0;
    writeGyroByte(BMI088_GYRO_INT_CTRL_REG, BMI088_GYRO_DRDY_ENABLE);

    delay(10);
    return testConnection() ? 1 : 0;
  }

  GyroDeviceType getType() const override
  {
    return GYRO_BMI088;
  }

  int readGyro(VectorInt16& v) override
  {
    uint8_t buffer[6];
    if (!readGyroBytes(BMI088_GYRO_DATA_REG, 6, buffer)) return 0;

    v.x = (((int16_t)buffer[1]) << 8) | buffer[0];
    v.y = (((int16_t)buffer[3]) << 8) | buffer[2];
    v.z = (((int16_t)buffer[5]) << 8) | buffer[4];
    return 1;
  }

  int readAccel(VectorInt16& v) override
  {
    uint8_t buffer[6];
    if (!readAccelBytes(BMI088_ACCEL_DATA_REG, 6, buffer)) return 0;

    int16_t x = (((int16_t)buffer[1]) << 8) | buffer[0];
    int16_t y = (((int16_t)buffer[3]) << 8) | buffer[2];
    int16_t z = (((int16_t)buffer[5]) << 8) | buffer[4];

    // ESP-FC's accel scale assumes +/-16G. BMI088 supports +/-12G or +/-24G,
    // so report +/-12G samples in the existing internal +/-16G raw scale.
    v.x = (int16_t)(((int32_t)x * 3) / 4);
    v.y = (int16_t)(((int32_t)y * 3) / 4);
    v.z = (int16_t)(((int32_t)z * 3) / 4);
    return 1;
  }

  void setDLPFMode(uint8_t mode) override
  {
    (void)mode;
  }

  int getRate() const override
  {
    return 2000;
  }

  void setRate(int rate) override
  {
    (void)rate;
  }

  bool testConnection() override
  {
    uint8_t accelWhoAmI = 0;
    uint8_t gyroWhoAmI = 0;
    return readAccelBytes(BMI088_ACCEL_CHIP_ID_REG, 1, &accelWhoAmI) &&
           readGyroBytes(BMI088_GYRO_CHIP_ID_REG, 1, &gyroWhoAmI) &&
           accelWhoAmI == BMI088_ACCEL_CHIP_ID &&
           gyroWhoAmI == BMI088_GYRO_CHIP_ID;
  }

private:
  bool readAccelBytes(uint8_t reg, uint8_t length, uint8_t* data)
  {
    if (_bus->isSPI())
    {
      if (length > sizeof(_buffer) - 1) return false;
      if (_bus->read(_addr, reg, length + 1, _buffer) != length + 1) return false;
      for (uint8_t i = 0; i < length; i++) data[i] = _buffer[i + 1];
      return true;
    }
    return _bus->read(_addr, reg, length, data) == length;
  }

  bool readGyroBytes(uint8_t reg, uint8_t length, uint8_t* data)
  {
    return _bus->read(_gyroAddr, reg, length, data) == length;
  }

  bool writeAccelByte(uint8_t reg, uint8_t value)
  {
    return _bus->writeByte(_addr, reg, value);
  }

  bool writeGyroByte(uint8_t reg, uint8_t value)
  {
    return _bus->writeByte(_gyroAddr, reg, value);
  }

  uint8_t _gyroAddr;
  uint8_t _buffer[10];
};

} // namespace Espfc::Device
