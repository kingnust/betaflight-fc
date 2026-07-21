#include "Sensor/AuxSensor.hpp"

#include "Hardware.h"
#include <cmath>

#if defined(ESPFC_DRONE_PROTO_SONAR_DEBUG)
#define SONAR_DEBUG_LINE(v) do { Serial0.print("SONAR DEBUG: "); Serial0.println(v); Serial0.flush(); } while(0)
#define SONAR_DEBUG_VALUE(k, v) do { Serial0.print("SONAR DEBUG: "); Serial0.print(k); Serial0.print("="); Serial0.println(v); Serial0.flush(); } while(0)
#define SONAR_DEBUG_HEX(k, v) do { Serial0.print("SONAR DEBUG: "); Serial0.print(k); Serial0.print("=0x"); Serial0.println(v, HEX); Serial0.flush(); } while(0)
#else
#define SONAR_DEBUG_LINE(v)
#define SONAR_DEBUG_VALUE(k, v)
#define SONAR_DEBUG_HEX(k, v)
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
namespace {

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
constexpr uint32_t PMW3901_BOOT_DELAY_MS = 1000;
constexpr uint32_t PMW3901_RETRY_MS = 2000;
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_MTF02P)
constexpr uint8_t MTF02P_FRAME_HEAD = 0xEF;
constexpr uint8_t MTF02P_MSG_ID_RANGE_SENSOR = 0x51;
constexpr uint8_t MTF02P_HEADER_LEN = 6;
constexpr uint8_t MTF02P_MAX_PAYLOAD_LEN = 64;
constexpr uint8_t MTF02P_RANGE_PAYLOAD_LEN = 20;
constexpr uint8_t MTF02P_MSP_HEADER_LEN = 8;
constexpr uint8_t MTF02P_MSP_MAX_PAYLOAD_LEN = 16;
constexpr uint16_t MTF02P_MSP_RANGEFINDER = 0x1F01;
constexpr uint16_t MTF02P_MSP_OPTICAL_FLOW = 0x1F02;
constexpr uint32_t MTF02P_STALE_MS = 1000;
constexpr size_t MTF02P_MAX_BYTES_PER_UPDATE = 160;

uint16_t readU16Le(const uint8_t* data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

int16_t readI16Le(const uint8_t* data)
{
  return (int16_t)readU16Le(data);
}

uint32_t readU32Le(const uint8_t* data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

int32_t readI32Le(const uint8_t* data)
{
  return (int32_t)readU32Le(data);
}

int16_t clampI16(int32_t value)
{
  if (value < -32768) return -32768;
  if (value > 32767) return 32767;
  return (int16_t)value;
}

uint8_t crc8DvbS2(uint8_t crc, uint8_t value)
{
  crc ^= value;
  for (uint8_t bit = 0; bit < 8; bit++)
  {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
TwoWire& rangefinderWire()
{
  return Wire;
}

constexpr uint32_t VL53L1X_I2C_SPEED = 100000;
constexpr uint8_t VL53L1X_ADDRESS = 0x29;
constexpr uint16_t VL53L1X_MODEL_ID = 0xEACC;
constexpr uint16_t VL53L1X_MODEL_ID_COMPAT = 0xEAAA;
constexpr uint32_t VL53L1X_BOOT_DELAY_MS = 1500;
constexpr uint32_t VL53L1X_INIT_RETRY_MS = 3000;
constexpr uint16_t VL53L1X_I2C_TIMEOUT_MS = 100;
constexpr uint16_t VL53L1X_SENSOR_TIMEOUT_MS = 500;
constexpr uint32_t VL53L1X_UPDATE_MS = 500;
constexpr uint32_t VL53L1X_MEASUREMENT_BUDGET_US = 50000;
constexpr uint8_t VL53L1X_MAX_FAILURES = 4;
constexpr uint32_t VL53L1X_STALE_MS = 2500;
constexpr uint32_t VL53L1X_TASK_STACK_BYTES = 8192;
constexpr uint32_t VL53L1X_RETRY_DISABLED = 0xffffffffu;

bool rangefinderAddressPresent(TwoWire& bus)
{
  SONAR_DEBUG_LINE("address probe begin");
  bus.beginTransmission(VL53L1X_ADDRESS);
  const uint8_t status = bus.endTransmission();
  SONAR_DEBUG_VALUE("address probe status", status);
  return status == 0;
}

bool rangefinderReadModelId(TwoWire& bus, uint16_t& modelId)
{
  SONAR_DEBUG_LINE("model id begin");
  bus.beginTransmission(VL53L1X_ADDRESS);
  bus.write((uint8_t)(VL53L1X::IDENTIFICATION__MODEL_ID >> 8));
  bus.write((uint8_t)VL53L1X::IDENTIFICATION__MODEL_ID);
  const uint8_t writeStatus = bus.endTransmission();
  SONAR_DEBUG_VALUE("model id write status", writeStatus);
  if (writeStatus != 0)
  {
    return false;
  }

  const uint8_t readCount = bus.requestFrom(VL53L1X_ADDRESS, (uint8_t)2);
  SONAR_DEBUG_VALUE("model id read count", readCount);
  if (readCount != 2)
  {
    return false;
  }

  modelId = ((uint16_t)bus.read() << 8) | bus.read();
  SONAR_DEBUG_HEX("model id", modelId);
  return true;
}
#endif

} // namespace
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X) || defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
namespace {

int16_t debugClamp(uint32_t value)
{
  return value > 32767u ? 32767 : (int16_t)value;
}

} // namespace
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
namespace {

uint8_t classifyColor(const Espfc::Device::ColorTCS34725Data& color)
{
  const uint32_t rawTotal = (uint32_t)color.red + color.green + color.blue;
  if (color.clear < 90 || rawTotal < 160)
  {
    return Espfc::COLOR_SENSOR_DARK;
  }

  // Rough white balance for the TCS34725 + onboard LED. Without this, green
  // tends to dominate and blue/purple are hard to reach.
  const uint32_t r = (uint32_t)color.red * 100u;
  const uint32_t g = (uint32_t)color.green * 82u;
  const uint32_t b = (uint32_t)color.blue * 135u;

  uint32_t maxValue = r;
  if (g > maxValue) maxValue = g;
  if (b > maxValue) maxValue = b;
  uint32_t minValue = r;
  if (g < minValue) minValue = g;
  if (b < minValue) minValue = b;
  if (maxValue == 0)
  {
    return Espfc::COLOR_SENSOR_DARK;
  }

  const uint32_t chroma = maxValue - minValue;
  const uint32_t satPct = (chroma * 100u) / maxValue;

  if (satPct < 24u)
  {
    if (color.clear > 2200 || rawTotal > 2600)
    {
      return Espfc::COLOR_SENSOR_WHITE;
    }
    return Espfc::COLOR_SENSOR_GREY;
  }

  int32_t hue = 0;
  if (r >= g && r >= b)
  {
    hue = (int32_t)(60l * ((int32_t)g - (int32_t)b) / (int32_t)chroma);
    if (hue < 0) hue += 360;
  }
  else if (g >= r && g >= b)
  {
    hue = 120 + (int32_t)(60l * ((int32_t)b - (int32_t)r) / (int32_t)chroma);
  }
  else
  {
    hue = 240 + (int32_t)(60l * ((int32_t)r - (int32_t)g) / (int32_t)chroma);
  }
  if (hue < 0) hue += 360;
  if (hue >= 360) hue -= 360;

  if (hue < 12 || hue >= 345) return Espfc::COLOR_SENSOR_RED;
  if (hue < 28)
  {
    return color.clear < 1700 ? Espfc::COLOR_SENSOR_BROWN : Espfc::COLOR_SENSOR_ORANGE;
  }
  if (hue < 52)
  {
    return color.clear < 1500 ? Espfc::COLOR_SENSOR_BROWN : Espfc::COLOR_SENSOR_ORANGE;
  }
  if (hue < 76) return Espfc::COLOR_SENSOR_YELLOW;
  if (hue < 165) return Espfc::COLOR_SENSOR_GREEN;
  if (hue < 205) return Espfc::COLOR_SENSOR_CYAN;
  if (hue < 255) return Espfc::COLOR_SENSOR_BLUE;
  if (hue < 305) return Espfc::COLOR_SENSOR_PURPLE;
  if (hue < 345) return Espfc::COLOR_SENSOR_MAGENTA;
  return Espfc::COLOR_SENSOR_UNKNOWN;
}

const __FlashStringHelper* colorTypeName(uint8_t type)
{
  switch(type)
  {
    case Espfc::COLOR_SENSOR_DARK: return F("black");
    case Espfc::COLOR_SENSOR_WHITE: return F("white");
    case Espfc::COLOR_SENSOR_RED: return F("red");
    case Espfc::COLOR_SENSOR_GREEN: return F("green");
    case Espfc::COLOR_SENSOR_BLUE: return F("blue");
    case Espfc::COLOR_SENSOR_YELLOW: return F("yellow");
    case Espfc::COLOR_SENSOR_CYAN: return F("cyan");
    case Espfc::COLOR_SENSOR_MAGENTA: return F("magenta");
    case Espfc::COLOR_SENSOR_ORANGE: return F("orange");
    case Espfc::COLOR_SENSOR_PURPLE: return F("purple");
    case Espfc::COLOR_SENSOR_BROWN: return F("brown");
    case Espfc::COLOR_SENSOR_GREY: return F("grey");
    case Espfc::COLOR_SENSOR_UNKNOWN:
    default: return F("unknown");
  }
}

#if defined(ESPFC_DRONE_PROTO_COLOR_SERIAL_PRINT)
void printColorSensorStatus(const __FlashStringHelper* status)
{
  Serial.print(F("COLOR sensor="));
  Serial.println(status);
  Serial.flush();
}
#endif

} // namespace
#endif

namespace Espfc::Sensor {

AuxSensor::AuxSensor(Model& model): _model(model) {}

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
bool AuxSensor::beginOpticalFlow(uint32_t now)
{
  _lastFlowInitMs = now + PMW3901_RETRY_MS;
  if (Device::BusSPI* spi = Device::getMainSpiBus())
  {
    _model.state.aux.flow.present = _flow.begin(spi, ESPFC_PMW3901_CS);
    _model.state.aux.flow.chipId = _flow.getChipId();
    _model.state.aux.flow.inverseChipId = _flow.getInverseChipId();
    if (_model.state.aux.flow.present)
    {
      _model.logger.info().log(F("AUX PMW3901")).logln(F("Y"));
    }
    return _model.state.aux.flow.present;
  }
  return false;
}
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_MTF02P)
void AuxSensor::resetMtf02pParser()
{
  _mtf02pFrameIndex = 0;
  _mtf02pFrameLength = 0;
  _mtf02pChecksum = 0;
}

void AuxSensor::resetMtf02pMspParser()
{
  _mtf02pMspFrameIndex = 0;
  _mtf02pMspPayloadLength = 0;
  _mtf02pMspCrc = 0;
}

bool AuxSensor::beginMtf02p(uint32_t now)
{
  (void)now;
  resetMtf02pParser();
  resetMtf02pMspParser();
  _mtf02pProtocol = 0;
  _model.state.aux.mtf02p.enabled = true;
  _model.state.aux.mtf02p.present = false;
  _model.state.aux.mtf02p.protocol = 0;
  _model.state.aux.mtf02p.rxByteCount = 0;
  _model.state.aux.mtf02p.syncByteCount = 0;
  _model.state.aux.mtf02p.noiseByteCount = 0;
  _model.state.aux.mtf02p.micolinkHeadCount = 0;
  _model.state.aux.mtf02p.mavlinkV1HeadCount = 0;
  _model.state.aux.mtf02p.mavlinkV2HeadCount = 0;
  _model.state.aux.mtf02p.mspHeadCount = 0;
  _model.state.aux.mtf02p.rawSampleIndex = 0;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = 255;
  _model.state.aux.flow.present = false;
  _model.state.aux.flow.chipId = 0x02;
  _model.state.aux.flow.inverseChipId = 0xFD;
  ESPFC_MTF02P_DEV.begin(ESPFC_MTF02P_BAUD, SERIAL_8N1, ESPFC_MTF02P_RX, ESPFC_MTF02P_TX);
  _mtf02pStarted = true;
  _model.logger.info().log(F("AUX MTF02P UART")).log(ESPFC_MTF02P_BAUD).log(F(" rx=")).log(ESPFC_MTF02P_RX).log(F(" tx=")).logln(ESPFC_MTF02P_TX);
  return true;
}

bool AuxSensor::handleMtf02pFrame(uint32_t now)
{
  const uint8_t msgId = _mtf02pFrame[3];
  const uint8_t payloadLen = _mtf02pFrame[5];

  if (msgId != MTF02P_MSG_ID_RANGE_SENSOR)
  {
    return false;
  }
  if (payloadLen < MTF02P_RANGE_PAYLOAD_LEN)
  {
    _model.state.aux.mtf02p.frameErrorCount++;
    return false;
  }

  const uint8_t* payload = &_mtf02pFrame[MTF02P_HEADER_LEN];
  const uint32_t sensorTimeMs = readU32Le(&payload[0]);
  const uint32_t distanceMmRaw = readU32Le(&payload[4]);
  const uint8_t strength = payload[8];
  const uint8_t precision = payload[9];
  const uint8_t tofStatus = payload[10];
  const int16_t flowVelX = readI16Le(&payload[12]);
  const int16_t flowVelY = readI16Le(&payload[14]);
  const uint8_t flowQuality = payload[16];
  const uint8_t flowStatus = payload[17];

  _model.state.aux.mtf02p.present = true;
  _model.state.aux.mtf02p.devId = _mtf02pFrame[1];
  _model.state.aux.mtf02p.sysId = _mtf02pFrame[2];
  _model.state.aux.mtf02p.seq = _mtf02pFrame[4];
  _model.state.aux.mtf02p.strength = strength;
  _model.state.aux.mtf02p.precision = precision;
  _model.state.aux.mtf02p.tofStatus = tofStatus;
  _model.state.aux.mtf02p.flowQuality = flowQuality;
  _model.state.aux.mtf02p.flowStatus = flowStatus;
  _model.state.aux.mtf02p.sensorTimeMs = sensorTimeMs;
  _model.state.aux.mtf02p.packetCount++;
  _model.state.aux.mtf02p.lastUpdate = now;

  _model.state.aux.range.present = true;
  _model.state.aux.range.distanceMm = distanceMmRaw > 65535u ? 65535u : (uint16_t)distanceMmRaw;
  _model.state.aux.range.status = tofStatus;
  _model.state.aux.range.signal = strength;
  _model.state.aux.range.ambient = precision;
  _model.state.aux.range.lastUpdate = now;
  _model.state.aux.range.readCount++;

  _model.state.aux.flow.present = true;
  _model.state.aux.flow.deltaX = flowVelX;
  _model.state.aux.flow.deltaY = flowVelY;
  _model.state.aux.flow.frameCount++;
  _model.state.aux.flow.lastUpdate = now;

  if (_model.config.debug.mode == DEBUG_RANGEFINDER || _model.config.debug.mode == DEBUG_LIDAR_TF)
  {
    _model.state.debug[0] = _model.state.aux.range.distanceMm;
    _model.state.debug[1] = _model.state.aux.range.status;
    _model.state.debug[2] = flowVelX;
    _model.state.debug[3] = flowVelY;
  }
  if (_model.config.debug.mode == DEBUG_RANGEFINDER_QUALITY)
  {
    _model.state.debug[0] = flowVelX;
    _model.state.debug[1] = flowVelY;
    _model.state.debug[6] = _model.state.aux.range.distanceMm;
    _model.state.debug[7] = _model.state.aux.range.status;
  }

  return true;
}

bool AuxSensor::parseMtf02pByte(uint8_t value, uint32_t now)
{
  if (_mtf02pFrameIndex == 0)
  {
    if (value != MTF02P_FRAME_HEAD)
    {
      _model.state.aux.mtf02p.noiseByteCount++;
      return false;
    }
    _mtf02pFrame[0] = value;
    _mtf02pChecksum = value;
    _mtf02pFrameIndex = 1;
    _model.state.aux.mtf02p.syncByteCount++;
    return false;
  }

  if (_mtf02pFrameIndex < MTF02P_HEADER_LEN)
  {
    _mtf02pFrame[_mtf02pFrameIndex] = value;
    _mtf02pChecksum += value;
    _mtf02pFrameIndex++;
    if (_mtf02pFrameIndex == MTF02P_HEADER_LEN)
    {
      _mtf02pFrameLength = _mtf02pFrame[5];
      if (_mtf02pFrameLength > MTF02P_MAX_PAYLOAD_LEN)
      {
        _model.state.aux.mtf02p.frameErrorCount++;
        resetMtf02pParser();
      }
    }
    return false;
  }

  const uint8_t checksumIndex = MTF02P_HEADER_LEN + _mtf02pFrameLength;
  if (_mtf02pFrameIndex < checksumIndex)
  {
    _mtf02pFrame[_mtf02pFrameIndex] = value;
    _mtf02pChecksum += value;
    _mtf02pFrameIndex++;
    return false;
  }

  const bool checksumOk = _mtf02pChecksum == value;
  bool handled = false;
  if (checksumOk)
  {
    handled = handleMtf02pFrame(now);
  }
  else
  {
    _model.state.aux.mtf02p.checksumErrorCount++;
  }
  resetMtf02pParser();
  return handled;
}

bool AuxSensor::handleMtf02pMspFrame(uint32_t now)
{
  const uint16_t command = readU16Le(&_mtf02pMspFrame[4]);
  const uint8_t* payload = &_mtf02pMspFrame[MTF02P_MSP_HEADER_LEN];

  if (command == MTF02P_MSP_RANGEFINDER)
  {
    if (_mtf02pMspPayloadLength != 5)
    {
      _model.state.aux.mtf02p.frameErrorCount++;
      return false;
    }
    const uint8_t quality = payload[0];
    const uint8_t status = quality > 0 ? 0 : 1;
    const uint32_t distanceMm = readU32Le(&payload[1]);
    _model.state.aux.mtf02p.strength = quality;
    _model.state.aux.mtf02p.tofStatus = status;
    _model.state.aux.range.present = true;
    _model.state.aux.range.distanceMm = distanceMm > 65535u ? 65535u : (uint16_t)distanceMm;
    _model.state.aux.range.status = status;
    _model.state.aux.range.signal = quality;
    _model.state.aux.range.lastUpdate = now;
    _model.state.aux.range.readCount++;
  }
  else if (command == MTF02P_MSP_OPTICAL_FLOW)
  {
    if (_mtf02pMspPayloadLength != 9)
    {
      _model.state.aux.mtf02p.frameErrorCount++;
      return false;
    }
    const uint8_t quality = payload[0];
    const uint8_t status = quality > 0 ? 0 : 1;
    _model.state.aux.mtf02p.flowQuality = quality;
    _model.state.aux.mtf02p.flowStatus = status;
    _model.state.aux.flow.present = true;
    _model.state.aux.flow.deltaX = clampI16(readI32Le(&payload[1]));
    _model.state.aux.flow.deltaY = clampI16(readI32Le(&payload[5]));
    _model.state.aux.flow.frameCount++;
    _model.state.aux.flow.lastUpdate = now;
  }
  else
  {
    return false;
  }

  _model.state.aux.mtf02p.present = true;
  _model.state.aux.mtf02p.packetCount++;
  _model.state.aux.mtf02p.lastUpdate = now;
  return true;
}

bool AuxSensor::parseMtf02pMspByte(uint8_t value, uint32_t now)
{
  if (_mtf02pMspFrameIndex == 0)
  {
    if (value != '$') return false;
    _mtf02pMspFrame[0] = value;
    _mtf02pMspFrameIndex = 1;
    return false;
  }

  if (_mtf02pMspFrameIndex == 1)
  {
    if (value == 'X')
    {
      _mtf02pMspFrame[1] = value;
      _mtf02pMspFrameIndex = 2;
    }
    else
    {
      resetMtf02pMspParser();
      if (value == '$')
      {
        _mtf02pMspFrame[0] = value;
        _mtf02pMspFrameIndex = 1;
      }
    }
    return false;
  }

  if (_mtf02pMspFrameIndex == 2)
  {
    if (value == '<')
    {
      _mtf02pMspFrame[2] = value;
      _mtf02pMspFrameIndex = 3;
      _model.state.aux.mtf02p.syncByteCount++;
    }
    else
    {
      resetMtf02pMspParser();
    }
    return false;
  }

  if (_mtf02pMspFrameIndex < MTF02P_MSP_HEADER_LEN)
  {
    _mtf02pMspFrame[_mtf02pMspFrameIndex++] = value;
    _mtf02pMspCrc = crc8DvbS2(_mtf02pMspCrc, value);
    if (_mtf02pMspFrameIndex == MTF02P_MSP_HEADER_LEN)
    {
      const uint16_t payloadLength = readU16Le(&_mtf02pMspFrame[6]);
      if (payloadLength > MTF02P_MSP_MAX_PAYLOAD_LEN)
      {
        _model.state.aux.mtf02p.frameErrorCount++;
        resetMtf02pMspParser();
      }
      else
      {
        _mtf02pMspPayloadLength = (uint8_t)payloadLength;
      }
    }
    return false;
  }

  const uint8_t checksumIndex = MTF02P_MSP_HEADER_LEN + _mtf02pMspPayloadLength;
  if (_mtf02pMspFrameIndex < checksumIndex)
  {
    _mtf02pMspFrame[_mtf02pMspFrameIndex++] = value;
    _mtf02pMspCrc = crc8DvbS2(_mtf02pMspCrc, value);
    return false;
  }

  bool handled = false;
  if (_mtf02pMspCrc == value)
  {
    handled = handleMtf02pMspFrame(now);
  }
  else
  {
    _model.state.aux.mtf02p.checksumErrorCount++;
  }
  resetMtf02pMspParser();
  return handled;
}

bool AuxSensor::updateMtf02p(uint32_t now)
{
  if (!_mtf02pStarted)
  {
    beginMtf02p(now);
  }

  bool updated = false;
  size_t bytesRead = 0;
  while (ESPFC_MTF02P_DEV.available() > 0 && bytesRead < MTF02P_MAX_BYTES_PER_UPDATE)
  {
    _model.state.aux.mtf02p.rxByteCount++;
    const uint8_t value = (uint8_t)ESPFC_MTF02P_DEV.read();
    if (value == 0xEF) _model.state.aux.mtf02p.micolinkHeadCount++;
    if (value == 0xFE) _model.state.aux.mtf02p.mavlinkV1HeadCount++;
    if (value == 0xFD) _model.state.aux.mtf02p.mavlinkV2HeadCount++;
    if (value == '$') _model.state.aux.mtf02p.mspHeadCount++;
    const uint8_t sampleIndex = _model.state.aux.mtf02p.rawSampleIndex;
    _model.state.aux.mtf02p.rawSample[sampleIndex] = value;
    _model.state.aux.mtf02p.rawSampleIndex = (sampleIndex + 1) % sizeof(_model.state.aux.mtf02p.rawSample);
    const bool mspUpdated = _mtf02pProtocol != 1 && parseMtf02pMspByte(value, now);
    const bool micolinkUpdated = _mtf02pProtocol != 2 && parseMtf02pByte(value, now);
    if (mspUpdated)
    {
      _mtf02pProtocol = 2;
      _model.state.aux.mtf02p.protocol = 2;
      resetMtf02pParser();
    }
    else if (micolinkUpdated)
    {
      _mtf02pProtocol = 1;
      _model.state.aux.mtf02p.protocol = 1;
      resetMtf02pMspParser();
    }
    updated = mspUpdated || micolinkUpdated || updated;
    bytesRead++;
  }

  if (_model.state.aux.mtf02p.present && (uint32_t)(now - _model.state.aux.mtf02p.lastUpdate) > MTF02P_STALE_MS)
  {
    _model.state.aux.mtf02p.present = false;
    _model.state.aux.range.present = false;
    _model.state.aux.range.status = 240;
    _model.state.aux.range.lastError = 240;
    _model.state.aux.range.lastErrorMs = now;
    _model.state.aux.flow.present = false;
  }

  return updated;
}
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
void AuxSensor::recordRangefinderError(uint8_t status, uint32_t now)
{
  _model.state.aux.range.lastError = status;
  _model.state.aux.range.lastErrorMs = now;
  _model.state.aux.range.failureCount++;
}

void AuxSensor::stopRangefinder(uint8_t status, uint32_t now, bool retry)
{
  SONAR_DEBUG_VALUE("stop status", status);
  _rangeStarted = false;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = status;
  _model.state.aux.range.lastUpdate = 0;
  _lastRangeInitMs = retry ? now + VL53L1X_INIT_RETRY_MS : VL53L1X_RETRY_DISABLED;
  _rangeFailures = 0;
  recordRangefinderError(status, now);
  if (retry)
  {
    _model.state.aux.range.recoveryCount++;
  }
}

bool AuxSensor::beginRangefinder(uint32_t now)
{
  SONAR_DEBUG_LINE("begin start");
  _lastRangeInitMs = now;
  _rangeFailures = 0;
  _rangeStarted = false;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = 255;
  _model.state.aux.range.lastUpdate = 0;
  _model.state.aux.range.initCount++;
  TwoWire& bus = rangefinderWire();

  pinMode(ESPFC_VL53_I2C_SDA, INPUT_PULLUP);
  pinMode(ESPFC_VL53_I2C_SCL, INPUT_PULLUP);
  SONAR_DEBUG_LINE("before wire begin");
  bus.begin(ESPFC_VL53_I2C_SDA, ESPFC_VL53_I2C_SCL, VL53L1X_I2C_SPEED);
  SONAR_DEBUG_LINE("after wire begin");
  bus.setClock(VL53L1X_I2C_SPEED);
  SONAR_DEBUG_LINE("after set clock");
  bus.setTimeOut(VL53L1X_I2C_TIMEOUT_MS);
  SONAR_DEBUG_LINE("after set timeout");
  vTaskDelay(pdMS_TO_TICKS(10));

  if (!rangefinderAddressPresent(bus))
  {
    stopRangefinder(252, now, true);
    return false;
  }

  uint16_t modelId = 0;
  if (!rangefinderReadModelId(bus, modelId))
  {
    stopRangefinder(251, now, true);
    return false;
  }
  if (modelId != VL53L1X_MODEL_ID && modelId != VL53L1X_MODEL_ID_COMPAT)
  {
    stopRangefinder(250, now, true);
    return false;
  }

  _range.setBus(&bus);
  _range.setTimeout(VL53L1X_SENSOR_TIMEOUT_MS);
  SONAR_DEBUG_LINE("before vl init");
  if (!_range.init())
  {
    stopRangefinder(_range.timeoutOccurred() ? 248 : 249, now, true);
    return false;
  }
  SONAR_DEBUG_LINE("after vl init");

  SONAR_DEBUG_LINE("before distance mode");
  if (!_range.setDistanceMode(VL53L1X::Long))
  {
    stopRangefinder(247, now, true);
    return false;
  }
  SONAR_DEBUG_LINE("after distance mode");

  SONAR_DEBUG_LINE("before timing budget");
  if (!_range.setMeasurementTimingBudget(VL53L1X_MEASUREMENT_BUDGET_US))
  {
    stopRangefinder(246, now, true);
    return false;
  }
  SONAR_DEBUG_LINE("after timing budget");

  SONAR_DEBUG_LINE("before start continuous");
  _range.startContinuous(VL53L1X_UPDATE_MS);
  SONAR_DEBUG_LINE("after start continuous");
  if (_range.timeoutOccurred() || _range.last_status != 0)
  {
    stopRangefinder(245, now, true);
    return false;
  }

  _rangeStarted = true;
  _rangeStartedAtMs = now;
  _model.state.aux.range.present = true;
  _model.state.aux.range.status = 254;
  _model.logger.info().log(F("AUX VL53L1X")).logln(F("Y"));
  return true;
}

bool AuxSensor::rangefinderFailure(uint8_t status, uint32_t now)
{
  _model.state.aux.range.status = status;
  _rangeFailures++;
  if (_rangeFailures >= VL53L1X_MAX_FAILURES)
  {
    stopRangefinder(status, now, true);
    return false;
  }
  recordRangefinderError(status, now);
  return false;
}

bool AuxSensor::updateRangefinder(uint32_t now)
{
  if (!_rangeStarted || !_model.state.aux.range.present)
  {
    return false;
  }

  if (!_range.dataReady())
  {
    if (_range.last_status != 0)
    {
      return rangefinderFailure(244, now);
    }
    const uint32_t freshnessBase = _model.state.aux.range.lastUpdate ? _model.state.aux.range.lastUpdate : _rangeStartedAtMs;
    if (freshnessBase && now - freshnessBase > VL53L1X_STALE_MS)
    {
      return rangefinderFailure(240, now);
    }
    return false;
  }

  SONAR_DEBUG_LINE("before read");
  const uint32_t readStart = millis();
  const uint16_t distance = _range.read(false);
  SONAR_DEBUG_VALUE("after read", distance);
  if (_range.timeoutOccurred() || _range.last_status != 0)
  {
    return rangefinderFailure(242, now);
  }
  if (millis() - readStart > VL53L1X_SENSOR_TIMEOUT_MS)
  {
    return rangefinderFailure(243, now);
  }

  _model.state.aux.range.distanceMm = distance;
  _model.state.aux.range.present = true;
  _model.state.aux.range.status = (uint8_t)_range.ranging_data.range_status;
  _model.state.aux.range.signal = debugClamp(lrintf(_range.ranging_data.peak_signal_count_rate_MCPS * 1000.0f));
  _model.state.aux.range.ambient = debugClamp(lrintf(_range.ranging_data.ambient_count_rate_MCPS * 1000.0f));
  _model.state.aux.range.lastUpdate = now;
  _model.state.aux.range.readCount++;
  _rangeFailures = 0;

  if (_model.config.debug.mode == DEBUG_RANGEFINDER || _model.config.debug.mode == DEBUG_LIDAR_TF)
  {
    _model.state.debug[0] = distance;
    _model.state.debug[1] = _model.state.aux.range.status;
  }
  if (_model.config.debug.mode == DEBUG_RANGEFINDER_QUALITY)
  {
    _model.state.debug[0] = _model.state.aux.range.status;
    _model.state.debug[1] = _model.state.aux.range.signal;
    _model.state.debug[2] = _model.state.aux.range.ambient;
  }

  return true;
}

void AuxSensor::rangefinderTaskEntry(void* arg)
{
  static_cast<AuxSensor*>(arg)->rangefinderTask();
  vTaskDelete(nullptr);
}

void AuxSensor::rangefinderTask()
{
  SONAR_DEBUG_LINE("task boot delay");
  vTaskDelay(pdMS_TO_TICKS(VL53L1X_BOOT_DELAY_MS));
  SONAR_DEBUG_LINE("task start");
  _lastRangeInitMs = 0;

  for (;;)
  {
    const uint32_t now = millis();
    _model.state.aux.range.taskHeartbeatMs = now;

    if (!_rangeStarted)
    {
      if (_lastRangeInitMs == VL53L1X_RETRY_DISABLED)
      {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      if (_lastRangeInitMs == 0 || now >= _lastRangeInitMs)
      {
        beginRangefinder(now);
      }
      vTaskDelay(pdMS_TO_TICKS(_rangeStarted ? VL53L1X_UPDATE_MS : 250));
      continue;
    }

    updateRangefinder(now);
    vTaskDelay(pdMS_TO_TICKS(VL53L1X_UPDATE_MS));
  }
}
#endif

int AuxSensor::begin()
{
#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
#if defined(ESPFC_DRONE_PROTO_AUX_MTF02P)
  beginMtf02p(millis());
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901) && defined(ESPFC_SPI_0)
  _model.state.aux.flow.present = false;
  _model.state.aux.flow.chipId = 0;
  _model.state.aux.flow.inverseChipId = 0;
  _lastFlowInitMs = millis() + PMW3901_BOOT_DELAY_MS;
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725) && defined(ESPFC_I2C_0)
  if (Device::BusI2C* i2c = Device::getMainI2cBus())
  {
    _model.state.aux.color.present = _color.begin(i2c, ESPFC_TCS_LED_PIN);
    _model.state.aux.color.ledOn = _model.state.aux.color.present;
    _model.logger.info().log(F("AUX TCS34725")).logln(_model.state.aux.color.present ? "Y" : "");
#if defined(ESPFC_DRONE_PROTO_COLOR_SERIAL_PRINT)
    printColorSensorStatus(_model.state.aux.color.present ? F("present") : F("missing"));
#endif
  }
#if defined(ESPFC_DRONE_PROTO_COLOR_SERIAL_PRINT)
  else
  {
    printColorSensorStatus(F("i2c-missing"));
  }
#endif
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_VL53L1X)
  SONAR_DEBUG_LINE("begin task create");
  _lastRangeInitMs = millis() + VL53L1X_BOOT_DELAY_MS;
  _model.state.aux.range.present = false;
  _model.state.aux.range.status = 253;
  _model.state.aux.range.lastError = 255;
  _model.state.aux.range.lastErrorMs = 0;
  _model.state.aux.range.taskHeartbeatMs = 0;
  if (!_rangeTask)
  {
    const BaseType_t started = xTaskCreate(rangefinderTaskEntry, "vl53Task", VL53L1X_TASK_STACK_BYTES, this, 0, &_rangeTask);
    SONAR_DEBUG_VALUE("task create", started);
    _model.logger.info().log(F("AUX VL53L1X")).logln(started == pdPASS ? "TASK" : "TASK FAIL");
    if (started != pdPASS)
    {
      _model.state.aux.range.status = 241;
      recordRangefinderError(241, millis());
    }
  }
#endif
#endif

  return 1;
}

int AuxSensor::update()
{
#if defined(ESPFC_DRONE_PROTO_AUX_ENABLED)
#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901) || defined(ESPFC_DRONE_PROTO_AUX_MTF02P) || defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
  const uint32_t now = millis();
#endif
  int updated = 0;

#if defined(ESPFC_DRONE_PROTO_AUX_MTF02P)
  if (updateMtf02p(now))
  {
    updated = 1;
  }
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_PMW3901)
  if (!_model.state.aux.flow.present && now >= _lastFlowInitMs)
  {
    beginOpticalFlow(now);
  }

  if (_model.state.aux.flow.present && now - _lastFlowMs >= 20)
  {
    _lastFlowMs = now;
    int16_t dx = 0;
    int16_t dy = 0;
    if (_flow.readMotion(dx, dy))
    {
      _model.state.aux.flow.deltaX = dx;
      _model.state.aux.flow.deltaY = dy;
      _model.state.aux.flow.frameCount++;
      _model.state.aux.flow.lastUpdate = now;
      if (_model.config.debug.mode == DEBUG_RANGEFINDER || _model.config.debug.mode == DEBUG_LIDAR_TF)
      {
        _model.state.debug[2] = dx;
        _model.state.debug[3] = dy;
      }
      updated = 1;
    }
  }
#endif

#if defined(ESPFC_DRONE_PROTO_AUX_TCS34725)
  if (!_model.state.aux.color.present)
  {
#if defined(ESPFC_DRONE_PROTO_COLOR_SERIAL_PRINT)
    if (now - _lastColorPrintMs >= 1000)
    {
      _lastColorPrintMs = now;
      printColorSensorStatus(F("missing"));
    }
#endif
  }
  else if (now - _lastColorMs >= 700)
  {
    _lastColorMs = now;
    Device::ColorTCS34725Data color;
    if (_color.read(color))
    {
      _model.state.aux.color.clear = color.clear;
      _model.state.aux.color.red = color.red;
      _model.state.aux.color.green = color.green;
      _model.state.aux.color.blue = color.blue;
      _model.state.aux.color.type = classifyColor(color);
      _model.state.aux.color.lastUpdate = now;
      if (_model.config.debug.mode == DEBUG_ADC_INTERNAL)
      {
        _model.state.debug[0] = debugClamp(color.red);
        _model.state.debug[1] = debugClamp(color.green);
        _model.state.debug[2] = debugClamp(color.blue);
        _model.state.debug[3] = debugClamp(color.clear);
      }
#if defined(ESPFC_DRONE_PROTO_COLOR_SERIAL_PRINT)
      if (now - _lastColorPrintMs >= 1000)
      {
        _lastColorPrintMs = now;
        Serial.print(F("COLOR "));
        Serial.print(colorTypeName(_model.state.aux.color.type));
        Serial.print(F(" r="));
        Serial.print(_model.state.aux.color.red);
        Serial.print(F(" g="));
        Serial.print(_model.state.aux.color.green);
        Serial.print(F(" b="));
        Serial.print(_model.state.aux.color.blue);
        Serial.print(F(" clear="));
        Serial.println(_model.state.aux.color.clear);
        Serial.flush();
      }
#endif
      updated = 1;
    }
#if defined(ESPFC_DRONE_PROTO_COLOR_SERIAL_PRINT)
    else if (now - _lastColorPrintMs >= 1000)
    {
      _lastColorPrintMs = now;
      printColorSensorStatus(F("waiting"));
    }
#endif
  }
#endif

  return updated;
#else
  return 0;
#endif
}

} // namespace Espfc::Sensor
