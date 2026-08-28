#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_ENABLE_WK2132)

#include "Device/Wk2132Serial.hpp"

#include <algorithm>

namespace Espfc::Device {
namespace {

constexpr uint8_t REG_GENA = 0x00;
constexpr uint8_t REG_GRST = 0x01;
constexpr uint8_t REG_SPAGE = 0x03;
constexpr uint8_t REG_SCR = 0x04;
constexpr uint8_t REG_LCR = 0x05;
constexpr uint8_t REG_FCR = 0x06;
constexpr uint8_t REG_SIER = 0x07;
constexpr uint8_t REG_TFCNT = 0x09;
constexpr uint8_t REG_RFCNT = 0x0a;
constexpr uint8_t REG_FSR = 0x0b;

constexpr uint8_t REG_BAUD1 = 0x04;
constexpr uint8_t REG_BAUD0 = 0x05;
constexpr uint8_t REG_PRES = 0x06;
constexpr uint8_t REG_RFTL = 0x07;
constexpr uint8_t REG_TFTL = 0x08;

constexpr uint8_t PAGE_0 = 0;
constexpr uint8_t PAGE_1 = 1;
constexpr uint8_t FSR_TX_DATA = 0x04;
constexpr uint8_t FSR_RX_DATA = 0x08;
constexpr uint8_t FSR_RX_PARITY_ERROR = 0x10;
constexpr uint8_t FSR_RX_FRAME_ERROR = 0x20;
constexpr uint8_t FSR_RX_BREAK_ERROR = 0x40;
constexpr uint8_t FSR_RX_OVERFLOW_ERROR = 0x80;
constexpr size_t I2C_FIFO_CHUNK = 32;
constexpr uint32_t FLUSH_TIMEOUT_MS = 100;

uint8_t lineControlValue(const SerialDeviceConfig& config)
{
  uint8_t value = config.stop_bits == SDC_SERIAL_STOP_BITS_2 ? 0x01 : 0x00;
  if(config.parity == SDC_SERIAL_PARITY_ODD)
  {
    value |= 0x0a;
  }
  else if(config.parity == SDC_SERIAL_PARITY_EVEN)
  {
    value |= 0x0c;
  }
  return value;
}

bool supportedSerialConfig(const SerialDeviceConfig& config)
{
  const bool stopBitsSupported =
    config.stop_bits == SDC_SERIAL_STOP_BITS_1 ||
    config.stop_bits == SDC_SERIAL_STOP_BITS_2;
  const bool paritySupported =
    config.parity == SDC_SERIAL_PARITY_NONE ||
    config.parity == SDC_SERIAL_PARITY_ODD ||
    config.parity == SDC_SERIAL_PARITY_EVEN;
  return config.data_bits == 8 && stopBitsSupported && paritySupported && !config.inverted;
}

} // namespace

Wk2132SerialPort::Wk2132SerialPort(Wk2132Bridge& bridge, uint8_t channel):
  _bridge(bridge), _channel(channel)
{
}

void Wk2132SerialPort::begin(const SerialDeviceConfig& config)
{
  _bridge.configurePort(*this, config);
}

void Wk2132SerialPort::updateBaudRate(int baud)
{
  SerialDeviceConfig config;
  config.baud = baud > 0 ? static_cast<uint32_t>(baud) : 0;
  begin(config);
}

int Wk2132SerialPort::available()
{
  if(!_bridge.present() || !_stats.configured || !_bridge.tryLock())
  {
    return _peeked >= 0 ? 1 : 0;
  }

  uint16_t count = 0;
  const bool ok = _bridge.readFifoCountUnlocked(*this, true, count);
  _bridge.unlock();
  return ok ? static_cast<int>(count) + (_peeked >= 0 ? 1 : 0) : (_peeked >= 0 ? 1 : 0);
}

int Wk2132SerialPort::read()
{
  uint8_t value = 0;
  return readMany(&value, 1) == 1 ? value : -1;
}

size_t Wk2132SerialPort::readMany(uint8_t* data, size_t length)
{
  if(data == nullptr || length == 0 || !_stats.configured)
  {
    return 0;
  }

  size_t copied = 0;
  if(_peeked >= 0)
  {
    data[copied++] = static_cast<uint8_t>(_peeked);
    _peeked = -1;
    if(copied == length)
    {
      return copied;
    }
  }

  if(!_bridge.present() || !_bridge.tryLock())
  {
    return copied;
  }

  uint16_t availableCount = 0;
  if(_bridge.readFifoCountUnlocked(*this, true, availableCount))
  {
    const size_t requested = std::min(length - copied, static_cast<size_t>(availableCount));
    const size_t received = _bridge.readFifoUnlocked(*this, data + copied, requested);
    copied += received;
    _stats.rxBytes += received;
  }
  _bridge.unlock();
  return copied;
}

int Wk2132SerialPort::peek()
{
  if(_peeked >= 0)
  {
    return _peeked;
  }
  if(!_bridge.present() || !_stats.configured || !_bridge.tryLock())
  {
    return -1;
  }

  uint16_t count = 0;
  uint8_t value = 0;
  if(_bridge.readFifoCountUnlocked(*this, true, count) && count > 0 &&
     _bridge.readFifoUnlocked(*this, &value, 1) == 1)
  {
    _peeked = value;
    _stats.rxBytes++;
  }
  _bridge.unlock();
  return _peeked;
}

void Wk2132SerialPort::flush()
{
  if(!_bridge.present() || !_stats.configured)
  {
    return;
  }

  const uint32_t startedAt = millis();
  while(static_cast<uint32_t>(millis() - startedAt) < FLUSH_TIMEOUT_MS)
  {
    if(isTxFifoEmpty())
    {
      return;
    }
    delay(1);
  }
  _stats.flushTimeouts++;
}

size_t Wk2132SerialPort::write(uint8_t value)
{
  return write(&value, 1);
}

size_t Wk2132SerialPort::write(const uint8_t* data, size_t length)
{
  if(data == nullptr || length == 0 || !_bridge.present() || !_stats.configured || !_bridge.tryLock())
  {
    return 0;
  }

  uint16_t used = 0;
  size_t written = 0;
  if(_bridge.readFifoCountUnlocked(*this, false, used))
  {
    const size_t writable = std::min(length, static_cast<size_t>(256u - used));
    written = _bridge.writeFifoUnlocked(*this, data, writable);
    _stats.txBytes += written;
  }
  _bridge.unlock();
  return written;
}

int Wk2132SerialPort::availableForWrite()
{
  if(!_bridge.present() || !_stats.configured || !_bridge.tryLock())
  {
    return 0;
  }

  uint16_t used = 0;
  const bool ok = _bridge.readFifoCountUnlocked(*this, false, used);
  _bridge.unlock();
  return ok ? static_cast<int>(256u - used) : 0;
}

bool Wk2132SerialPort::isTxFifoEmpty()
{
  if(!_bridge.present() || !_stats.configured || !_bridge.tryLock())
  {
    return true;
  }

  uint16_t used = 0;
  const bool ok = _bridge.readFifoCountUnlocked(*this, false, used);
  _bridge.unlock();
  return !ok || used == 0;
}

bool Wk2132SerialPort::isSoft() const
{
  return false;
}

Wk2132SerialPort::operator bool() const
{
  return _bridge.present() && _stats.configured;
}

uint8_t Wk2132SerialPort::channel() const
{
  return _channel;
}

const Wk2132PortStats& Wk2132SerialPort::stats() const
{
  return _stats;
}

bool Wk2132SerialPort::fifoLevels(uint16_t& rxUsed, uint16_t& txUsed, uint8_t& fifoStatus)
{
  rxUsed = 0;
  txUsed = 0;
  fifoStatus = 0;
  if(!_bridge.present() || !_stats.configured || !_bridge.lock())
  {
    return false;
  }

  const bool rxOk = _bridge.readFifoCountUnlocked(*this, true, rxUsed, &fifoStatus);
  const bool txOk = _bridge.readFifoCountUnlocked(*this, false, txUsed, &fifoStatus);
  _bridge.unlock();
  return rxOk && txOk;
}

bool Wk2132SerialPort::clearFifos()
{
  if(!_bridge.present() || !_stats.configured || !_bridge.lock())
  {
    return false;
  }
  const bool ok = _bridge.clearFifosUnlocked(*this);
  if(ok)
  {
    _peeked = -1;
    _lastFifoStatus = 0;
  }
  _bridge.unlock();
  return ok;
}

void Wk2132SerialPort::recordFifoStatus(uint8_t value)
{
  const uint8_t risingErrors = static_cast<uint8_t>((value & 0xf0u) & ~(_lastFifoStatus & 0xf0u));
  if(risingErrors & FSR_RX_OVERFLOW_ERROR) _stats.rxOverflowErrors++;
  if(risingErrors & FSR_RX_PARITY_ERROR) _stats.parityErrors++;
  if(risingErrors & FSR_RX_FRAME_ERROR) _stats.framingErrors++;
  if(risingErrors & FSR_RX_BREAK_ERROR) _stats.breakErrors++;
  _lastFifoStatus = value;
}

Wk2132Bridge::Wk2132Bridge(WireClass& wire):
  _wire(wire),
  _camera(*this, Wk2132Protocol::CHANNEL_CAMERA),
  _uwb(*this, Wk2132Protocol::CHANNEL_UWB)
{
}

bool Wk2132Bridge::begin(const Wk2132BridgeConfig& config)
{
  _stats.beginAttempts++;
  const bool previouslyOwnedBus = _config.ownsI2cBus;
  _config = config;
  _present = false;
  _i2cModeValid = false;
  _camera._stats.configured = false;
  _camera._peeked = -1;
  _camera._lastFifoStatus = 0;
  _uwb._stats.configured = false;
  _uwb._peeked = -1;
  _uwb._lastFifoStatus = 0;

  if(_mutex == nullptr)
  {
    _mutex = xSemaphoreCreateMutex();
  }
  if(_mutex == nullptr || config.sda < 0 || config.scl < 0 ||
     config.i2cFrequencyHz == 0 || config.oscillatorHz == 0)
  {
    return false;
  }

  if(!lock(100))
  {
    return false;
  }

  if(_started && previouslyOwnedBus)
  {
#if !defined(ESPFC_I2C_0_SOFT)
    _wire.end();
#endif
    _started = false;
  }

  if(config.ownsI2cBus)
  {
#if defined(ESPFC_I2C_0_SOFT)
    _wire.begin(config.sda, config.scl, config.i2cFrequencyHz);
    _wire.setTimeout(config.i2cTimeoutMs);
#else
    if(!_wire.begin(config.sda, config.scl, config.i2cFrequencyHz))
    {
      recordBusResult(-1);
      unlock();
      return false;
    }
    _wire.setTimeOut(config.i2cTimeoutMs);
#endif
  }

  // A shared bus has already been initialized by Hardware::begin(). Never
  // reconfigure or stop it here because the barometer, magnetometer, and color
  // sensor use the same controller and pins.
  _started = true;

  if(config.reset >= 0)
  {
    pinMode(config.reset, OUTPUT);
    digitalWrite(config.reset, LOW);
    delay(2);
    digitalWrite(config.reset, HIGH);
    delay(10);
  }
  if(config.irq >= 0)
  {
    pinMode(config.irq, INPUT_PULLUP);
  }

  uint8_t gena = 0;
  bool ok = readRegisterUnlocked(Wk2132Protocol::CHANNEL_CAMERA, REG_GENA, gena);
  if(ok)
  {
    _stats.gena = gena;
    _i2cModeValid = (gena & 0xc0u) == 0x80u;
    ok = _i2cModeValid;
  }
  if(ok)
  {
    ok = writeVerifyUnlocked(
      Wk2132Protocol::CHANNEL_CAMERA, REG_GENA, static_cast<uint8_t>((gena & 0xfcu) | 0x03u), 0x03u);
  }
  if(ok)
  {
    ok = writeRegisterUnlocked(Wk2132Protocol::CHANNEL_CAMERA, REG_GRST, 0x03u);
    delay(1);
  }

  _present = ok;
  unlock();
  return _present;
}

void Wk2132Bridge::end()
{
  _present = false;
  _i2cModeValid = false;
  _camera._stats.configured = false;
  _uwb._stats.configured = false;
  if(_mutex != nullptr && !lock(100))
  {
    return;
  }
  if(_started && _config.ownsI2cBus)
  {
#if !defined(ESPFC_I2C_0_SOFT)
    _wire.end();
#endif
  }
  _started = false;
  if(_mutex != nullptr)
  {
    unlock();
  }
}

bool Wk2132Bridge::started() const
{
  return _started;
}

bool Wk2132Bridge::present() const
{
  return _present;
}

bool Wk2132Bridge::i2cModeValid() const
{
  return _i2cModeValid;
}

const Wk2132BridgeConfig& Wk2132Bridge::config() const
{
  return _config;
}

const Wk2132BridgeStats& Wk2132Bridge::stats() const
{
  return _stats;
}

uint8_t Wk2132Bridge::registerAddress(uint8_t channel) const
{
  return Wk2132Protocol::i2cAddress(_config.ia1High, _config.ia0High, channel, false);
}

uint8_t Wk2132Bridge::fifoAddress(uint8_t channel) const
{
  return Wk2132Protocol::i2cAddress(_config.ia1High, _config.ia0High, channel, true);
}

Wk2132SerialPort& Wk2132Bridge::cameraPort()
{
  return _camera;
}

Wk2132SerialPort& Wk2132Bridge::uwbPort()
{
  return _uwb;
}

bool Wk2132Bridge::lock(uint32_t timeoutMs)
{
  return _mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

bool Wk2132Bridge::tryLock()
{
  return _mutex != nullptr && xSemaphoreTake(_mutex, 0) == pdTRUE;
}

void Wk2132Bridge::unlock()
{
  if(_mutex != nullptr)
  {
    xSemaphoreGive(_mutex);
  }
}

bool Wk2132Bridge::configurePort(Wk2132SerialPort& port, const SerialDeviceConfig& config)
{
  port._stats.requestedBaud = config.baud;
  port._stats.configured = false;
  port._peeked = -1;
  port._lastFifoStatus = 0;

  const Wk2132Protocol::BaudConfig baud =
    Wk2132Protocol::calculateBaud(_config.oscillatorHz, config.baud);
  if(!_present || !supportedSerialConfig(config) || !baud.valid || !lock())
  {
    port._stats.configErrors++;
    return false;
  }

  bool ok = setPageUnlocked(port._channel, PAGE_0);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_SCR, 0x00u);
  if(ok) ok = setPageUnlocked(port._channel, PAGE_1);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_BAUD1, static_cast<uint8_t>(baud.divisorRegister >> 8));
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_BAUD0, static_cast<uint8_t>(baud.divisorRegister));
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_PRES, baud.prescaler, 0x0fu);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_RFTL, 0x00u);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_TFTL, 0x00u);
  if(ok) ok = setPageUnlocked(port._channel, PAGE_0);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_LCR, lineControlValue(config), 0x3fu);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_SIER, 0x00u);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_FCR, 0x0fu, 0x0cu);
  if(ok) ok = writeVerifyUnlocked(port._channel, REG_SCR, 0x03u, 0x03u);

  port._stats.configured = ok;
  port._stats.actualBaud = baud.actualBaud;
  port._stats.baudErrorPpm = baud.errorPpm;
  if(!ok) port._stats.configErrors++;
  unlock();
  return ok;
}

bool Wk2132Bridge::clearFifosUnlocked(Wk2132SerialPort& port)
{
  if(!setPageUnlocked(port._channel, PAGE_0))
  {
    return false;
  }
  return writeVerifyUnlocked(port._channel, REG_FCR, 0x0fu, 0x0cu);
}

bool Wk2132Bridge::readRegisterUnlocked(uint8_t channel, uint8_t reg, uint8_t& value)
{
  const uint8_t address = registerAddress(channel);
  _wire.beginTransmission(address);
  if(_wire.write(reg) != 1)
  {
    recordBusResult(-2);
    return false;
  }
  const uint8_t writeStatus = _wire.endTransmission();
  if(writeStatus != 0)
  {
    recordBusResult(writeStatus);
    return false;
  }

  const uint8_t received = _wire.requestFrom(address, static_cast<uint8_t>(1));
  if(received != 1 || !_wire.available())
  {
    recordBusResult(-3);
    return false;
  }
  value = static_cast<uint8_t>(_wire.read());
  recordBusResult(0);
  return true;
}

bool Wk2132Bridge::writeRegisterUnlocked(uint8_t channel, uint8_t reg, uint8_t value)
{
  const uint8_t address = registerAddress(channel);
  _wire.beginTransmission(address);
  if(_wire.write(reg) != 1 || _wire.write(value) != 1)
  {
    recordBusResult(-2);
    return false;
  }
  const uint8_t status = _wire.endTransmission();
  recordBusResult(status);
  return status == 0;
}

bool Wk2132Bridge::writeVerifyUnlocked(uint8_t channel, uint8_t reg, uint8_t value, uint8_t mask)
{
  uint8_t readback = 0;
  if(!writeRegisterUnlocked(channel, reg, value) ||
     !readRegisterUnlocked(channel, reg, readback))
  {
    return false;
  }
  if((readback & mask) != (value & mask))
  {
    _stats.verificationErrors++;
    return false;
  }
  return true;
}

bool Wk2132Bridge::setPageUnlocked(uint8_t channel, uint8_t page)
{
  return writeVerifyUnlocked(channel, REG_SPAGE, page ? 0x01u : 0x00u, 0x01u);
}

bool Wk2132Bridge::readFifoCountUnlocked(
  Wk2132SerialPort& port, bool receive, uint16_t& count, uint8_t* fifoStatus)
{
  uint8_t rawCount = 0;
  uint8_t status = 0;
  if(!readRegisterUnlocked(port._channel, receive ? REG_RFCNT : REG_TFCNT, rawCount))
  {
    if(receive) port._stats.fifoReadErrors++;
    else port._stats.fifoWriteErrors++;
    return false;
  }

  const bool statusRequired = rawCount == 0 || fifoStatus != nullptr;
  if(statusRequired)
  {
    if(!readRegisterUnlocked(port._channel, REG_FSR, status))
    {
      if(receive) port._stats.fifoReadErrors++;
      else port._stats.fifoWriteErrors++;
      return false;
    }
    port.recordFifoStatus(status);
    if(fifoStatus != nullptr) *fifoStatus = status;
  }

  if(rawCount != 0)
  {
    count = rawCount;
  }
  else if(receive)
  {
    count = (status & FSR_RX_DATA) ? 256u : 0u;
  }
  else
  {
    count = (status & FSR_TX_DATA) ? 256u : 0u;
  }
  return true;
}

size_t Wk2132Bridge::readFifoUnlocked(Wk2132SerialPort& port, uint8_t* data, size_t length)
{
  size_t total = 0;
  const uint8_t address = fifoAddress(port._channel);
  while(total < length)
  {
    const uint8_t chunk = static_cast<uint8_t>(
      std::min(length - total, static_cast<size_t>(I2C_FIFO_CHUNK)));
    _wire.beginTransmission(address);
    const uint8_t selectStatus = _wire.endTransmission();
    if(selectStatus != 0)
    {
      recordBusResult(selectStatus);
      port._stats.fifoReadErrors++;
      break;
    }

    const uint8_t received = _wire.requestFrom(address, chunk);
    if(received != chunk)
    {
      recordBusResult(-3);
      port._stats.fifoReadErrors++;
    }
    size_t copied = 0;
    while(copied < received && _wire.available())
    {
      data[total + copied] = static_cast<uint8_t>(_wire.read());
      copied++;
    }
    total += copied;
    port._stats.rxTransactions++;
    if(copied != chunk)
    {
      break;
    }
    recordBusResult(0);
  }
  return total;
}

size_t Wk2132Bridge::writeFifoUnlocked(Wk2132SerialPort& port, const uint8_t* data, size_t length)
{
  size_t total = 0;
  const uint8_t address = fifoAddress(port._channel);
  while(total < length)
  {
    const size_t chunk = std::min(length - total, I2C_FIFO_CHUNK);
    _wire.beginTransmission(address);
    if(_wire.write(data + total, chunk) != chunk)
    {
      recordBusResult(-2);
      port._stats.fifoWriteErrors++;
      break;
    }
    const uint8_t status = _wire.endTransmission();
    if(status != 0)
    {
      recordBusResult(status);
      port._stats.fifoWriteErrors++;
      break;
    }
    total += chunk;
    port._stats.txTransactions++;
    recordBusResult(0);
  }
  return total;
}

void Wk2132Bridge::recordBusResult(int status)
{
  _stats.lastWireStatus = status;
  if(status != 0)
  {
    _stats.busErrors++;
  }
}

} // namespace Espfc::Device

#endif
