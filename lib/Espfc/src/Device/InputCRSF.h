#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include "Rc/Crsf.h"
#include "TelemetryManager.h"

// https://github.com/CapnBry/CRServoF/blob/master/lib/CrsfSerial/crsf_protocol.h
// https://github.com/AlessioMorale/crsf_parser/tree/master
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c

namespace Espfc::Device {

struct CrsfInputDiagnostics
{
    uint32_t activeBaud = 420000;
    uint32_t baudSwitches = 0;
    uint32_t rawBytes = 0;
    uint32_t syncBytes = 0;
    uint32_t validFrames = 0;
    uint32_t crcErrors = 0;
    uint32_t invalidSizes = 0;
    uint32_t unsupportedTypes = 0;
    uint32_t rcFrames = 0;
    uint32_t subsetRcFrames = 0;
    uint32_t lastByteMs = 0;
    uint32_t lastValidFrameMs = 0;
    uint8_t lastFrameType = 0;
    bool baudLocked = false;
};

class InputCRSF: public InputDevice
{
  public:
    enum CrsfState {
      CRSF_ADDR,
      CRSF_SIZE,
      CRSF_TYPE,
      CRSF_DATA,
      CRSF_CRC
    };

    InputCRSF();

    int begin(Device::SerialDevice * serial, TelemetryManager * telemetry);
    virtual InputStatus update() override;
    virtual uint16_t get(uint8_t i) const override;
    virtual void get(uint16_t * data, size_t len) const override;
    virtual size_t getChannelCount() const override;
    virtual bool needAverage() const override;

    static const CrsfInputDiagnostics& diagnostics();

    void print(char c) const;
    void parse(Rc::CrsfMessage& frame, int d);

  private:
    void reset();
    void apply(const Rc::CrsfMessage& msg);
    void applyLinkStats(const Rc::CrsfMessage& msg);
    void applyChannels(const Rc::CrsfMessage& msg);
    void applyChannelsSubset(const Rc::CrsfMessage& msg);
    void applyMspReq(const Rc::CrsfMessage& msg);

    static constexpr size_t CHANNELS = 32;
    static constexpr size_t TELEMETRY_INTERVAL = 20000;
    static constexpr uint32_t CRSF_BAUD_DEFAULT = 420000;
    static constexpr uint32_t CRSF_BAUD_FALLBACK = 400000;
    static constexpr uint32_t BAUD_PROBE_INTERVAL_MS = 1200;
    static constexpr uint8_t VALID_FRAMES_TO_LOCK_BAUD = 2;

    Device::SerialDevice * _serial;
    TelemetryManager * _telemetry;
    CrsfState _state;
    uint8_t _idx;
    bool _new_data;
    Rc::CrsfMessage _frame;
    uint16_t _channels[CHANNELS];
    uint32_t _telemetry_next;
    uint32_t _lastBaudProbeMs;
    uint8_t _validFramesAtBaud;
    Connect::MspMessage _msg;

    static CrsfInputDiagnostics _diagnostics;
};

}
