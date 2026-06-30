#include "Target/Target.h"

#if defined(ESPFC_SPI_0)

#include "Device/OpticalFlow/OpticalFlowPMW3901.hpp"
#include "Hal/Gpio.h"
#include <Arduino.h>

namespace Espfc::Device {

bool OpticalFlowPMW3901::begin(BusSPI* bus, int8_t cs)
{
  if (!bus || cs == -1) return false;

  _bus = bus;
  _cs = cs;
  _present = false;

  Hal::Gpio::digitalWrite(_cs, HIGH);
  Hal::Gpio::pinMode(_cs, OUTPUT);

  auto& spi = _bus->getDevice();
  spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  Hal::Gpio::digitalWrite(_cs, HIGH);
  delay(1);
  Hal::Gpio::digitalWrite(_cs, LOW);
  delay(1);
  Hal::Gpio::digitalWrite(_cs, HIGH);
  delay(1);
  spi.endTransaction();

  writeReg(0x3A, 0x5A);
  delay(5);

  const uint8_t chipId = readReg(0x00);
  const uint8_t inverseChipId = readReg(0x5F);
  if (chipId != 0x49 && inverseChipId != 0xB8) return false;

  readReg(0x02);
  readReg(0x03);
  readReg(0x04);
  readReg(0x05);
  readReg(0x06);
  delay(1);

  initRegisters();
  _present = true;
  return true;
}

bool OpticalFlowPMW3901::readMotion(int16_t& deltaX, int16_t& deltaY)
{
  if (!_present) return false;

  readReg(0x02);
  const uint8_t xL = readReg(0x03);
  const uint8_t xH = readReg(0x04);
  const uint8_t yL = readReg(0x05);
  const uint8_t yH = readReg(0x06);
  deltaX = (int16_t)((uint16_t)xH << 8 | xL);
  deltaY = (int16_t)((uint16_t)yH << 8 | yL);
  return true;
}

void OpticalFlowPMW3901::writeReg(uint8_t reg, uint8_t value)
{
  if (!_bus || _cs == -1) return;

  auto& spi = _bus->getDevice();
  spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  Hal::Gpio::digitalWrite(_cs, LOW);
  delayMicroseconds(50);
  spi.transfer(reg | 0x80u);
  spi.transfer(value);
  delayMicroseconds(50);
  Hal::Gpio::digitalWrite(_cs, HIGH);
  spi.endTransaction();
  delayMicroseconds(200);
}

uint8_t OpticalFlowPMW3901::readReg(uint8_t reg)
{
  if (!_bus || _cs == -1) return 0;

  auto& spi = _bus->getDevice();
  spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  Hal::Gpio::digitalWrite(_cs, LOW);
  delayMicroseconds(50);
  spi.transfer(reg & ~0x80u);
  delayMicroseconds(50);
  const uint8_t value = spi.transfer(0);
  delayMicroseconds(100);
  Hal::Gpio::digitalWrite(_cs, HIGH);
  spi.endTransaction();
  return value;
}

void OpticalFlowPMW3901::initRegisters()
{
  writeReg(0x7F, 0x00);
  writeReg(0x61, 0xAD);
  writeReg(0x7F, 0x03);
  writeReg(0x40, 0x00);
  writeReg(0x7F, 0x05);
  writeReg(0x41, 0xB3);
  writeReg(0x43, 0xF1);
  writeReg(0x45, 0x14);
  writeReg(0x5B, 0x32);
  writeReg(0x5F, 0x34);
  writeReg(0x7B, 0x08);
  writeReg(0x7F, 0x06);
  writeReg(0x44, 0x1B);
  writeReg(0x40, 0xBF);
  writeReg(0x4E, 0x3F);
  writeReg(0x7F, 0x08);
  writeReg(0x65, 0x20);
  writeReg(0x6A, 0x18);
  writeReg(0x7F, 0x09);
  writeReg(0x4F, 0xAF);
  writeReg(0x5F, 0x40);
  writeReg(0x48, 0x80);
  writeReg(0x49, 0x80);
  writeReg(0x57, 0x77);
  writeReg(0x60, 0x78);
  writeReg(0x61, 0x78);
  writeReg(0x62, 0x08);
  writeReg(0x63, 0x50);
  writeReg(0x7F, 0x0A);
  writeReg(0x45, 0x60);
  writeReg(0x7F, 0x00);
  writeReg(0x4D, 0x11);
  writeReg(0x55, 0x80);
  writeReg(0x74, 0x1F);
  writeReg(0x75, 0x1F);
  writeReg(0x4A, 0x78);
  writeReg(0x4B, 0x78);
  writeReg(0x44, 0x08);
  writeReg(0x45, 0x50);
  writeReg(0x64, 0xFF);
  writeReg(0x65, 0x1F);
  writeReg(0x7F, 0x14);
  writeReg(0x65, 0x60);
  writeReg(0x66, 0x08);
  writeReg(0x63, 0x78);
  writeReg(0x7F, 0x15);
  writeReg(0x48, 0x58);
  writeReg(0x7F, 0x07);
  writeReg(0x41, 0x0D);
  writeReg(0x43, 0x14);
  writeReg(0x4B, 0x0E);
  writeReg(0x45, 0x0F);
  writeReg(0x44, 0x42);
  writeReg(0x4C, 0x80);
  writeReg(0x7F, 0x10);
  writeReg(0x5B, 0x02);
  writeReg(0x7F, 0x07);
  writeReg(0x40, 0x41);
  writeReg(0x70, 0x00);

  delay(100);
  writeReg(0x32, 0x44);
  writeReg(0x7F, 0x07);
  writeReg(0x40, 0x40);
  writeReg(0x7F, 0x06);
  writeReg(0x62, 0xF0);
  writeReg(0x63, 0x00);
  writeReg(0x7F, 0x0D);
  writeReg(0x48, 0xC0);
  writeReg(0x6F, 0xD5);
  writeReg(0x7F, 0x00);
  writeReg(0x5B, 0xA0);
  writeReg(0x4E, 0xA8);
  writeReg(0x5A, 0x50);
  writeReg(0x40, 0x80);
}

} // namespace Espfc::Device

#endif
