#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_ENABLE_WK2132)

#include "Device/DroneProtoWk2132.hpp"

#ifndef ESPFC_DRONE_PROTO_WK2132_SDA
#define ESPFC_DRONE_PROTO_WK2132_SDA 17
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_SCL
#define ESPFC_DRONE_PROTO_WK2132_SCL 16
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_RESET
#define ESPFC_DRONE_PROTO_WK2132_RESET -1
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_IRQ
#define ESPFC_DRONE_PROTO_WK2132_IRQ 6
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_IA1
#define ESPFC_DRONE_PROTO_WK2132_IA1 0
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_IA0
#define ESPFC_DRONE_PROTO_WK2132_IA0 1
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_I2C_HZ
#define ESPFC_DRONE_PROTO_WK2132_I2C_HZ 400000
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_I2C_TIMEOUT_MS
#define ESPFC_DRONE_PROTO_WK2132_I2C_TIMEOUT_MS 20
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_OSCILLATOR_HZ
#define ESPFC_DRONE_PROTO_WK2132_OSCILLATOR_HZ 14745600
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_CAMERA_BAUD
#define ESPFC_DRONE_PROTO_WK2132_CAMERA_BAUD 115200
#endif
#ifndef ESPFC_DRONE_PROTO_WK2132_UWB_BAUD
#define ESPFC_DRONE_PROTO_WK2132_UWB_BAUD 115200
#endif

namespace Espfc::Device::DroneProtoWk2132 {
namespace {

static_assert(ESPFC_DRONE_PROTO_WK2132_SDA == ESPFC_I2C_0_SDA,
  "WK2132 SDA must match the shared FC I2C bus");
static_assert(ESPFC_DRONE_PROTO_WK2132_SCL == ESPFC_I2C_0_SCL,
  "WK2132 SCL must match the shared FC I2C bus");

Wk2132Bridge s_bridge(WireInstance);

SerialDeviceConfig serialConfig(uint32_t baud)
{
  SerialDeviceConfig config;
  config.baud = baud;
  config.rx_pin = -1;
  config.tx_pin = -1;
  config.inverted = false;
  config.data_bits = 8;
  config.parity = SDC_SERIAL_PARITY_NONE;
  config.stop_bits = SDC_SERIAL_STOP_BITS_1;
  return config;
}

} // namespace

bool begin()
{
  Wk2132BridgeConfig config;
  config.sda = ESPFC_DRONE_PROTO_WK2132_SDA;
  config.scl = ESPFC_DRONE_PROTO_WK2132_SCL;
  config.reset = ESPFC_DRONE_PROTO_WK2132_RESET;
  config.irq = ESPFC_DRONE_PROTO_WK2132_IRQ;
  config.ia1High = ESPFC_DRONE_PROTO_WK2132_IA1 != 0;
  config.ia0High = ESPFC_DRONE_PROTO_WK2132_IA0 != 0;
  config.i2cFrequencyHz = ESPFC_DRONE_PROTO_WK2132_I2C_HZ;
  config.i2cTimeoutMs = ESPFC_DRONE_PROTO_WK2132_I2C_TIMEOUT_MS;
  config.oscillatorHz = ESPFC_DRONE_PROTO_WK2132_OSCILLATOR_HZ;
  config.ownsI2cBus = false;

  if(!s_bridge.begin(config))
  {
    return false;
  }

  s_bridge.cameraPort().begin(serialConfig(ESPFC_DRONE_PROTO_WK2132_CAMERA_BAUD));
  s_bridge.uwbPort().begin(serialConfig(ESPFC_DRONE_PROTO_WK2132_UWB_BAUD));
  return static_cast<bool>(s_bridge.cameraPort()) && static_cast<bool>(s_bridge.uwbPort());
}

Wk2132Bridge& bridge()
{
  return s_bridge;
}

Wk2132SerialPort& cameraPort()
{
  return s_bridge.cameraPort();
}

Wk2132SerialPort& uwbPort()
{
  return s_bridge.uwbPort();
}

} // namespace Espfc::Device::DroneProtoWk2132

#endif
