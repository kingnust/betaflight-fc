#pragma once

#include "Target/Target.h"

#if defined(ESPFC_SPI_0)

#include "Device/BusSPI.h"
#include <cstdint>

namespace Espfc::Device {

class OpticalFlowPMW3901
{
  public:
    bool begin(BusSPI* bus, int8_t cs);
    bool readMotion(int16_t& deltaX, int16_t& deltaY);
    bool isPresent() const { return _present; }
    uint8_t getChipId() const { return _chipId; }
    uint8_t getInverseChipId() const { return _inverseChipId; }

  private:
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void initRegisters();

    BusSPI* _bus = nullptr;
    int8_t _cs = -1;
    bool _present = false;
    uint8_t _chipId = 0;
    uint8_t _inverseChipId = 0;
};

} // namespace Espfc::Device

#endif
