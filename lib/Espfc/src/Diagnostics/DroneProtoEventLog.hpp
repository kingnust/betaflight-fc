#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Espfc {

class Model;

namespace Diagnostics {

enum class EventType : uint8_t
{
  SAMPLE = 0,
  CONTROL_SOURCE,
  TRAINER_REQUEST,
  TRAINER_ACTIVE,
  ARMING_FLAGS,
  RECEIVER_HEALTH,
  FAILSAFE,
  POSHOLD_RELEASE,
};

struct Event
{
  uint32_t timeMs = 0;
  EventType type = EventType::SAMPLE;
  int32_t a = 0;
  int32_t b = 0;
  int32_t c = 0;
};

constexpr size_t EVENT_LOG_CAPACITY = 64;

struct EventLogState
{
  Event entries[EVENT_LOG_CAPACITY];
  uint8_t next = 0;
  uint8_t count = 0;
  uint32_t dropped = 0;
};

class DroneProtoEventLog
{
  public:
    explicit DroneProtoEventLog(Model& model);
    void begin();
    void update(uint32_t nowMs);
    static const char * typeName(EventType type);

  private:
    void push(EventType type, int32_t a, int32_t b, int32_t c, uint32_t nowMs);

    Model& _model;
    uint32_t _nextSampleMs = 0;
    uint32_t _armingFlags = 0;
    uint8_t _source = 0;
    uint8_t _failsafe = 0;
    uint8_t _posholdRelease = 0;
    bool _trainerRequest = false;
    bool _trainerActive = false;
    bool _receiverHealthy = false;
};

}
}
