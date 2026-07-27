#pragma once

#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_ENABLE_WK2132)

#include "Device/Wk2132Serial.hpp"

namespace Espfc::Device::DroneProtoWk2132 {

bool begin();
Wk2132Bridge& bridge();
Wk2132SerialPort& cameraPort();
Wk2132SerialPort& uwbPort();

} // namespace Espfc::Device::DroneProtoWk2132

#endif
