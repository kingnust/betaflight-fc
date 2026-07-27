#pragma once

#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_ENABLE_WK2132)

#include "Device/SerialDevice.h"
#include "Device/Wk2132Protocol.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace Espfc::Device {

struct Wk2132BridgeConfig
{
  int8_t sda = -1;
  int8_t scl = -1;
  int8_t reset = -1;
  int8_t irq = -1;
  bool ia1High = true;
  bool ia0High = true;
  uint32_t i2cFrequencyHz = 400000;
  uint16_t i2cTimeoutMs = 20;
  uint32_t oscillatorHz = 14745600;
};

struct Wk2132BridgeStats
{
  uint32_t beginAttempts = 0;
  uint32_t busErrors = 0;
  uint32_t verificationErrors = 0;
  int16_t lastWireStatus = 0;
  uint8_t gena = 0;
};

struct Wk2132PortStats
{
  bool configured = false;
  uint32_t requestedBaud = 0;
  uint32_t actualBaud = 0;
  uint32_t baudErrorPpm = 0;
  uint32_t rxBytes = 0;
  uint32_t txBytes = 0;
  uint32_t rxTransactions = 0;
  uint32_t txTransactions = 0;
  uint32_t configErrors = 0;
  uint32_t fifoReadErrors = 0;
  uint32_t fifoWriteErrors = 0;
  uint32_t rxOverflowErrors = 0;
  uint32_t parityErrors = 0;
  uint32_t framingErrors = 0;
  uint32_t breakErrors = 0;
  uint32_t flushTimeouts = 0;
};

class Wk2132Bridge;

class Wk2132SerialPort final: public SerialDevice
{
public:
  void begin(const SerialDeviceConfig& conf) override;
  void updateBaudRate(int baud) override;
  int available() override;
  int read() override;
  size_t readMany(uint8_t* data, size_t length) override;
  int peek() override;
  void flush() override;
  size_t write(uint8_t value) override;
  size_t write(const uint8_t* data, size_t length) override;
  int availableForWrite() override;
  bool isTxFifoEmpty() override;
  bool isSoft() const override;
  operator bool() const override;
  using Print::write;

  uint8_t channel() const;
  const Wk2132PortStats& stats() const;
  bool fifoLevels(uint16_t& rxUsed, uint16_t& txUsed, uint8_t& fifoStatus);
  bool clearFifos();

private:
  friend class Wk2132Bridge;
  Wk2132SerialPort(Wk2132Bridge& bridge, uint8_t channel);
  void recordFifoStatus(uint8_t value);

  Wk2132Bridge& _bridge;
  uint8_t _channel;
  int16_t _peeked = -1;
  uint8_t _lastFifoStatus = 0;
  Wk2132PortStats _stats;
};

class Wk2132Bridge
{
public:
  explicit Wk2132Bridge(TwoWire& wire);

  bool begin(const Wk2132BridgeConfig& config);
  void end();
  bool started() const;
  bool present() const;
  bool i2cModeValid() const;
  const Wk2132BridgeConfig& config() const;
  const Wk2132BridgeStats& stats() const;
  uint8_t registerAddress(uint8_t channel) const;
  uint8_t fifoAddress(uint8_t channel) const;
  Wk2132SerialPort& cameraPort();
  Wk2132SerialPort& uwbPort();

private:
  friend class Wk2132SerialPort;

  bool lock(uint32_t timeoutMs = 25);
  void unlock();
  bool configurePort(Wk2132SerialPort& port, const SerialDeviceConfig& config);
  bool clearFifosUnlocked(Wk2132SerialPort& port);
  bool readRegisterUnlocked(uint8_t channel, uint8_t reg, uint8_t& value);
  bool writeRegisterUnlocked(uint8_t channel, uint8_t reg, uint8_t value);
  bool writeVerifyUnlocked(uint8_t channel, uint8_t reg, uint8_t value, uint8_t mask = 0xff);
  bool setPageUnlocked(uint8_t channel, uint8_t page);
  bool readFifoCountUnlocked(Wk2132SerialPort& port, bool receive, uint16_t& count, uint8_t* fifoStatus = nullptr);
  size_t readFifoUnlocked(Wk2132SerialPort& port, uint8_t* data, size_t length);
  size_t writeFifoUnlocked(Wk2132SerialPort& port, const uint8_t* data, size_t length);
  void recordBusResult(int status);

  TwoWire& _wire;
  SemaphoreHandle_t _mutex = nullptr;
  Wk2132BridgeConfig _config;
  Wk2132BridgeStats _stats;
  bool _started = false;
  bool _present = false;
  bool _i2cModeValid = false;
  Wk2132SerialPort _camera;
  Wk2132SerialPort _uwb;
};

} // namespace Espfc::Device

#endif
