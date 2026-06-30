#pragma once

#include "Target/Target.h"

#if defined(ESPFC_I2C_0)

#include "Device/BusI2C.h"
#include <cstdint>

namespace Espfc::Device {

struct ColorTCS34725Data
{
  uint16_t clear = 0;
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
};

class ColorTCS34725
{
  public:
    bool begin(BusI2C* bus, int8_t ledPin);
    bool read(ColorTCS34725Data& data);
    bool isPresent() const { return _present; }

  private:
    bool write8(uint8_t reg, uint8_t value);
    bool read8(uint8_t reg, uint8_t& value);
    bool read16(uint8_t reg, uint16_t& value);

    BusI2C* _bus = nullptr;
    int8_t _ledPin = -1;
    bool _present = false;
};

} // namespace Espfc::Device

#endif
