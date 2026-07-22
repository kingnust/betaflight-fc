#include <algorithm>
#include "InputCRSF.h"
#include "Utils/MemoryHelper.h"

namespace Espfc::Device {

using namespace Espfc::Rc;

static constexpr uint8_t CRSF_SUBSET_RC_STARTING_CHANNEL_BITS = 5;
static constexpr uint8_t CRSF_SUBSET_RC_STARTING_CHANNEL_MASK = 0x1F;
static constexpr uint8_t CRSF_SUBSET_RC_RES_CONFIGURATION_MASK = 0x03;

static inline uint16_t convertSubsetChannel(uint16_t value, uint8_t resolutionConfig)
{
  return 988 + (value >> resolutionConfig);
}

CrsfInputDiagnostics InputCRSF::_diagnostics;

InputCRSF::InputCRSF():
  _serial(NULL), _telemetry(NULL), _state(CRSF_ADDR), _idx(0), _new_data(false),
  _lastBaudProbeMs(0), _validFramesAtBaud(0) {}

int InputCRSF::begin(Device::SerialDevice * serial, TelemetryManager * telemetry)
{
  _serial = serial;
  _telemetry = telemetry;
  _telemetry_next = micros() + TELEMETRY_INTERVAL;
  _lastBaudProbeMs = millis();
  _validFramesAtBaud = 0;
  _diagnostics = CrsfInputDiagnostics{};
  _diagnostics.activeBaud = CRSF_BAUD_DEFAULT;
  std::fill_n((uint8_t*)&_frame, sizeof(_frame), 0);
  std::fill_n(_channels, CHANNELS, 1500);
  _channels[2] = 988;
  return 1;
}

InputStatus FAST_CODE_ATTR InputCRSF::update()
{
  if(!_serial) return INPUT_IDLE;

  size_t len = _serial->available();
  if(len)
  {
    uint8_t buff[64] = {0};
    len = std::min(len, sizeof(buff));
    len = _serial->readMany(buff, len);
    if(len) _diagnostics.lastByteMs = millis();
    size_t i = 0;
    while(i < len)
    {
      parse(_frame, buff[i++]);
    }
  }

  if(_telemetry && micros() > _telemetry_next)
  {
    _telemetry_next = micros() + TELEMETRY_INTERVAL;
    _telemetry->process(*_serial, TELEMETRY_PROTOCOL_CRSF);
  }

  if(_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }

  const uint32_t nowMs = millis();
  if(_diagnostics.baudLocked && nowMs - _diagnostics.lastValidFrameMs >= BAUD_LOCK_TIMEOUT_MS)
  {
    // USB resets and receiver power-up order can interrupt a previously locked
    // UART. Return to the known CRSF default and resume normal baud probing.
    _diagnostics.baudLocked = false;
    _diagnostics.lockLosses++;
    _diagnostics.activeBaud = CRSF_BAUD_DEFAULT;
    _validFramesAtBaud = 0;
    _lastBaudProbeMs = nowMs;
    reset();
    _serial->updateBaudRate(CRSF_BAUD_DEFAULT);
  }

  if(!_diagnostics.baudLocked && nowMs - _lastBaudProbeMs >= BAUD_PROBE_INTERVAL_MS)
  {
    _diagnostics.activeBaud = _diagnostics.activeBaud == CRSF_BAUD_DEFAULT
      ? CRSF_BAUD_FALLBACK
      : CRSF_BAUD_DEFAULT;
    _diagnostics.baudSwitches++;
    _validFramesAtBaud = 0;
    _lastBaudProbeMs = nowMs;
    reset();
    _serial->updateBaudRate(_diagnostics.activeBaud);
  }

  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputCRSF::get(uint8_t i) const
{
  return i < CHANNELS ? _channels[i] : 1500;
}

void FAST_CODE_ATTR InputCRSF::get(uint16_t * data, size_t len) const
{
  if(data == nullptr) return;
  const size_t copyLen = std::min(len, CHANNELS);
  std::copy_n(_channels, copyLen, data);
  std::fill_n(data + copyLen, len - copyLen, 1500);
}

size_t InputCRSF::getChannelCount() const { return CHANNELS; }

bool InputCRSF::needAverage() const { return false; }

const CrsfInputDiagnostics& InputCRSF::diagnostics()
{
  return _diagnostics;
}

void FAST_CODE_ATTR InputCRSF::parse(CrsfMessage& msg, int d)
{
  uint8_t *data = reinterpret_cast<uint8_t*>(&msg);
  uint8_t c = (uint8_t)(d & 0xff);
  _diagnostics.rawBytes++;
  switch(_state)
  {
    case CRSF_ADDR:
      if(c == CRSF_SYNC_BYTE)
      {
        _diagnostics.syncBytes++;
        data[_idx++] = c;
        _state = CRSF_SIZE;
      }
      break;
    case CRSF_SIZE:
      if(c >= 2 && c <= CRSF_FRAME_SIZE_MAX - 2) // allowed size is in range 2-62
      {
        data[_idx++] = c;
        _state = CRSF_TYPE;
      } else {
        _diagnostics.invalidSizes++;
        reset();
      }
      break;
    case CRSF_TYPE:
      if(c == CRSF_FRAMETYPE_RC_CHANNELS_PACKED || c == CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED || c == CRSF_FRAMETYPE_LINK_STATISTICS || c == CRSF_FRAMETYPE_MSP_REQ || c == CRSF_FRAMETYPE_MSP_WRITE)
      {
        data[_idx++] = c;
        if (msg.size > 2) {
          _state = CRSF_DATA;
        } else {
          _state = CRSF_CRC; // no payload, next byte is crc
        }
      } else {
        _diagnostics.unsupportedTypes++;
        reset();
      }
      break;
    case CRSF_DATA:
      data[_idx++] = c;
      if(_idx > msg.size) // _idx is incremented here and operator > accounts as size - 2
      {
        _state = CRSF_CRC;
      }
      break;
    case CRSF_CRC:
      data[_idx++] = c;
      reset();
      uint8_t crc = msg.crc();
      if(c == crc) {
        const uint32_t nowMs = millis();
        _diagnostics.validFrames++;
        _diagnostics.lastValidFrameMs = nowMs;
        _diagnostics.lastFrameType = msg.type;
        _lastBaudProbeMs = nowMs;
        if(_validFramesAtBaud < VALID_FRAMES_TO_LOCK_BAUD) _validFramesAtBaud++;
        if(_validFramesAtBaud >= VALID_FRAMES_TO_LOCK_BAUD) _diagnostics.baudLocked = true;
        apply(msg);
      } else {
        _diagnostics.crcErrors++;
      }
      break;
    }
}

void FAST_CODE_ATTR InputCRSF::reset()
{
  _state = CRSF_ADDR;
  _idx = 0;
}

void FAST_CODE_ATTR InputCRSF::apply(const CrsfMessage& msg)
{
  switch (msg.type)
  {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
      applyChannels(msg);
      break;

    case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
      applyChannelsSubset(msg);
      break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
      applyLinkStats(msg);
      break;

    case CRSF_FRAMETYPE_MSP_REQ:
    case CRSF_FRAMETYPE_MSP_WRITE:
      applyMspReq(msg);
      break;

    default:
      break;
  }
}

void FAST_CODE_ATTR InputCRSF::applyLinkStats(const CrsfMessage& msg)
{
  const auto * stats = reinterpret_cast<const CrsfLinkStats*>(msg.payload);
  (void)stats;
  // TODO:
}

void FAST_CODE_ATTR InputCRSF::applyChannels(const CrsfMessage& msg)
{
  const auto * data = reinterpret_cast<const CrsfData*>(msg.payload);
  Crsf::decodeRcDataShift8(_channels, data);
  //Crsf::decodeRcData(_channels, frame);
  _diagnostics.rcFrames++;
  _new_data = true;
}

void FAST_CODE_ATTR InputCRSF::applyChannelsSubset(const CrsfMessage& msg)
{
  // payload: [config][packed channel data...]
  const size_t payloadSize = msg.size - 2; // size includes type and crc
  if(payloadSize < 2) return;

  const uint8_t* payload = msg.payload;
  uint8_t config = payload[0];
  const uint8_t startChannel = config & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
  config >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

  const uint8_t resolutionConfig = config & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK; // 0:10b, 1:11b, 2:12b, 3:13b
  const uint8_t channelBits = 10 + resolutionConfig;
  const uint16_t channelMask = (1u << channelBits) - 1u;

  const size_t packedBytes = payloadSize - 1;
  const size_t channelCount = (packedBytes * 8) / channelBits;
  if(channelCount == 0 || startChannel >= CHANNELS) return;

  const size_t channelsToProcess = std::min(channelCount, CHANNELS - startChannel);

  uint8_t bitsMerged = 0;
  uint32_t readValue = 0;
  size_t readByteIndex = 1;

  for(size_t i = 0; i < channelsToProcess; i++)
  {
    while(bitsMerged < channelBits && readByteIndex < payloadSize)
    {
      readValue |= (static_cast<uint32_t>(payload[readByteIndex++]) << bitsMerged);
      bitsMerged += 8;
    }

    if(bitsMerged < channelBits) break;

    const uint16_t channelValue = readValue & channelMask;
    _channels[startChannel + i] = convertSubsetChannel(channelValue, resolutionConfig);
    readValue >>= channelBits;
    bitsMerged -= channelBits;
  }

  _diagnostics.rcFrames++;
  _diagnostics.subsetRcFrames++;
  _new_data = true;
}

void FAST_CODE_ATTR InputCRSF::applyMspReq(const CrsfMessage& frame)
{
  if(!_telemetry) return;

  uint8_t origin = 0;

  Crsf::decodeMsp(frame, _msg, origin);

  if(_msg.isCmd() && _msg.isReady())
  {
    _telemetry->processMsp(*_serial, TELEMETRY_PROTOCOL_CRSF, _msg, origin);
  }

  _telemetry_next = micros() + TELEMETRY_INTERVAL;
}

}

