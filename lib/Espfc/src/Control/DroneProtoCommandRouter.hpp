#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Espfc {

namespace Control {

enum class DroneProtoInputSource : uint8_t
{
  RADIOMASTER = 0,
  TRAINER_PHONE,
  WIFI_DIRECT,
};

enum class DroneProtoTaskCommand : uint8_t
{
  NONE = 0,
  GO_TO_PRESET_1,
  GO_TO_PRESET_2,
  RETURN_HOME,
  TASK_1,
  TASK_2,
};

constexpr size_t DRONE_PROTO_FUNCTION_CHANNELS = 6;

struct DroneProtoTaskRequest
{
  bool valid = false;
  DroneProtoTaskCommand command = DroneProtoTaskCommand::NONE;
  DroneProtoInputSource source = DroneProtoInputSource::RADIOMASTER;
  uint32_t sequence = 0;
  uint32_t receivedAtMs = 0;
  uint16_t functionUs[DRONE_PROTO_FUNCTION_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
};

struct DroneProtoCommandState
{
  DroneProtoInputSource source = DroneProtoInputSource::RADIOMASTER;
  DroneProtoTaskCommand selected = DroneProtoTaskCommand::NONE;
  DroneProtoTaskRequest pending;
  uint32_t requestSequence = 0;
  uint32_t overwrittenRequests = 0;
  uint16_t functionUs[DRONE_PROTO_FUNCTION_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
  uint16_t trainerMarkerUs = 1000;
  uint16_t trainerHeartbeatReferenceUs = 0;
  uint32_t trainerHeartbeatLastTransitionMs = 0;
  uint8_t trainerHeartbeatTransitions = 0;
  bool trainerHeartbeatFresh = false;
  bool executeInitialized = false;
  bool executeHigh = false;
  bool trainerTaskArmed = false;
};

class DroneProtoCommandRouter
{
  public:
    static void reset(DroneProtoCommandState& state);
    static void route(uint16_t *logicalChannels,
                      const uint16_t *transportChannels,
                      size_t channelCount,
                      bool directActive,
                      const uint16_t *trainerSidebandChannels,
                      size_t trainerSidebandChannelCount,
                      bool trainerSidebandFresh,
                      uint32_t nowMs,
                      DroneProtoCommandState& state);
    static bool consumePending(DroneProtoCommandState& state, DroneProtoTaskRequest& request);
    static const char * sourceName(DroneProtoInputSource source);
    static const char * taskName(DroneProtoTaskCommand task);
};

}

}
