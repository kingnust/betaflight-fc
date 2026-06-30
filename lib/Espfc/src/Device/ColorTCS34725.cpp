#include "Target/Target.h"

#if defined(ESPFC_I2C_0)

#include "Device/ColorTCS34725.hpp"
#include "Hal/Gpio.h"
#include <Arduino.h>

namespace {

constexpr uint8_t TCS34725_ADDRESS = 0x29;
constexpr uint8_t TCS34725_COMMAND_BIT = 0x80;
constexpr uint8_t TCS34725_ENABLE = 0x00;
constexpr uint8_t TCS34725_ENABLE_PON = 0x01;
constexpr uint8_t TCS34725_ENABLE_AEN = 0x02;
constexpr uint8_t TCS34725_ATIME = 0x01;
constexpr uint8_t TCS34725_CONTROL = 0x0F;
constexpr uint8_t TCS34725_ID = 0x12;
constexpr uint8_t TCS34725_STATUS = 0x13;
constexpr uint8_t TCS34725_STATUS_AVALID = 0x01;
constexpr uint8_t TCS34725_CDATAL = 0x14;
constexpr uint8_t TCS34725_RDATAL = 0x16;
constexpr uint8_t TCS34725_GDATAL = 0x18;
constexpr uint8_t TCS34725_BDATAL = 0x1A;

} // namespace

namespace Espfc::Device {

bool ColorTCS34725::begin(BusI2C* bus, int8_t ledPin)
{
  if (!bus) return false;

  _bus = bus;
  _ledPin = ledPin;
  _present = false;

  if (_ledPin != -1)
  {
    Hal::Gpio::pinMode(_ledPin, OUTPUT);
    Hal::Gpio::digitalWrite(_ledPin, HIGH);
  }

  uint8_t id = 0;
  if (!read8(TCS34725_ID, id)) return false;
  if (id != 0x4D && id != 0x44 && id != 0x10) return false;

  if (!write8(TCS34725_ATIME, 0x00)) return false;   // 614 ms integration, same as the prototype example.
  if (!write8(TCS34725_CONTROL, 0x00)) return false; // 1x gain.
  if (!write8(TCS34725_ENABLE, TCS34725_ENABLE_PON)) return false;
  delay(3);
  if (!write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) return false;

  _present = true;
  return true;
}

bool ColorTCS34725::read(ColorTCS34725Data& data)
{
  if (!_present) return false;

  uint8_t status = 0;
  if (!read8(TCS34725_STATUS, status)) return false;
  if ((status & TCS34725_STATUS_AVALID) == 0) return false;

  return read16(TCS34725_CDATAL, data.clear) &&
         read16(TCS34725_RDATAL, data.red) &&
         read16(TCS34725_GDATAL, data.green) &&
         read16(TCS34725_BDATAL, data.blue);
}

bool ColorTCS34725::write8(uint8_t reg, uint8_t value)
{
  return _bus && _bus->write(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg, 1, &value);
}

bool ColorTCS34725::read8(uint8_t reg, uint8_t& value)
{
  return _bus && _bus->read(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg, 1, &value) == 1;
}

bool ColorTCS34725::read16(uint8_t reg, uint16_t& value)
{
  uint8_t data[2] = {0, 0};
  if (!_bus || _bus->read(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg, 2, data) != 2) return false;
  value = (uint16_t)data[1] << 8 | data[0];
  return true;
}

} // namespace Espfc::Device

#endif
