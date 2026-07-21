#include "DroneProtoProfiles.hpp"

#include "Control/Rates.h"
#include "Output/Mixers.h"
#include <cstring>

namespace Espfc::DroneProtoProfiles {

namespace {

void setProfileName(ModelConfig& config, const char* value)
{
  std::memset(config.modelName, 0, MODEL_NAME_LEN + 1);
  std::strncpy(config.modelName, value, MODEL_NAME_LEN);
}

void clearModes(ModelConfig& config)
{
  for (size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
  {
    config.conditions[i] = ActuatorCondition();
  }
}

void setMode(ActuatorCondition& condition, FlightMode mode, Axis channel, int16_t min, int16_t max)
{
  condition.id = mode;
  condition.ch = channel;
  condition.min = min;
  condition.max = max;
  condition.logicMode = 0;
  condition.linkId = 0;
}

void setCommonModes(ModelConfig& config)
{
  clearModes(config);
  setMode(config.conditions[0], MODE_ARMED, AXIS_AUX_1, 1700, 2100);
  setMode(config.conditions[1], MODE_ANGLE, AXIS_AUX_2, 1300, 2100);
  setMode(config.conditions[2], MODE_AIRMODE, AXIS_AUX_3, 1700, 2100);
#if defined(ESPFC_DRONE_PROTO_ENABLE_MTF02P)
  setMode(config.conditions[3], MODE_POSHOLD, AXIS_AUX_2, 1700, 2100);
#endif
}

void setCommonReceiver(ModelConfig& config)
{
  config.featureMask = FEATURE_RX_SERIAL;
  config.input.serialRxProvider = SERIALRX_CRSF;
  config.input.minRc = 875;
  config.input.maxRc = 2125;
  config.input.deadband = 5;
}

void setCommonDshot300(ModelConfig& config)
{
  config.output.protocol = ESC_PROTOCOL_DSHOT300;
  config.output.async = false;
  config.output.rate = 500;
  config.output.minCommand = 1000;
  config.output.minThrottle = 1070;
  config.output.maxThrottle = 2000;
  config.output.dshotIdle = 550;
#if defined(ESPFC_DRONE_PROTO_ENABLE_DSHOT_BIDIR)
  config.output.dshotTelemetry = true;
#else
  config.output.dshotTelemetry = false;
#endif
  config.gyro.rpmFilter.harmonics = 0;
  config.gyro.rpmFilter.minFreq = 100;
  config.gyro.rpmFilter.q = 500;
  config.gyro.rpmFilter.freqLpf = 150;
  config.gyro.rpmFilter.weights[0] = 100;
  config.gyro.rpmFilter.weights[1] = 100;
  config.gyro.rpmFilter.weights[2] = 100;
  config.gyro.rpmFilter.fade = 30;
}

void setHoverTune(ModelConfig& config)
{
  config.input.rateType = RATES_TYPE_ACTUAL;
  config.input.rate[AXIS_ROLL] = 12;
  config.input.rate[AXIS_PITCH] = 12;
  config.input.rate[AXIS_YAW] = 14;
  config.input.superRate[AXIS_ROLL] = 35;
  config.input.superRate[AXIS_PITCH] = 35;
  config.input.superRate[AXIS_YAW] = 30;
  config.input.expo[AXIS_ROLL] = 20;
  config.input.expo[AXIS_PITCH] = 20;
  config.input.expo[AXIS_YAW] = 15;
  config.input.rateLimit[AXIS_ROLL] = 350;
  config.input.rateLimit[AXIS_PITCH] = 350;
  config.input.rateLimit[AXIS_YAW] = 300;

  config.pid[FC_PID_ROLL] = PidConfig{38, 75, 20, 55};
  config.pid[FC_PID_PITCH] = PidConfig{40, 80, 22, 58};
  config.pid[FC_PID_YAW] = PidConfig{36, 75, 0, 45};
  config.pid[FC_PID_LEVEL] = PidConfig{38, 0, 0, 0};

  config.level.angleLimit = 35;
  config.level.rateLimit = 220;
  config.dterm.filter = FilterConfig(FILTER_PT1, 90);
  config.dterm.filter2 = FilterConfig(FILTER_PT1, 120);
  config.yaw.filter = FilterConfig(FILTER_PT1, 70);
  config.controller.tpaScale = 20;
  config.controller.tpaBreakpoint = 1550;
  config.iterm.limit = 25;
  config.iterm.relaxCutoff = 12;
  config.arming.smallAngle = 25;
}

void setBenchSafe(ModelConfig& config)
{
  setProfileName(config, name(PROFILE_BENCH_SAFE));
  setCommonReceiver(config);
  setCommonModes(config);
  setHoverTune(config);

  config.input.rate[AXIS_ROLL] = 10;
  config.input.rate[AXIS_PITCH] = 10;
  config.input.rate[AXIS_YAW] = 12;
  config.input.superRate[AXIS_ROLL] = 25;
  config.input.superRate[AXIS_PITCH] = 25;
  config.input.superRate[AXIS_YAW] = 20;
  config.input.rateLimit[AXIS_ROLL] = 250;
  config.input.rateLimit[AXIS_PITCH] = 250;
  config.input.rateLimit[AXIS_YAW] = 200;
  config.level.angleLimit = 30;
  config.level.rateLimit = 180;

  config.output.protocol = ESC_PROTOCOL_DISABLED;
  config.output.dshotTelemetry = false;
  config.gyro.rpmFilter.harmonics = 0;
  config.output.throttleLimitType = THROTTLE_LIMIT_TYPE_SCALE;
  config.output.throttleLimitPercent = 50;
  config.output.motorLimit = 70;
}

void setHoverSafe(ModelConfig& config)
{
  setProfileName(config, name(PROFILE_HOVER_SAFE));
  setCommonReceiver(config);
  setCommonModes(config);
  setHoverTune(config);
  setCommonDshot300(config);

  config.output.throttleLimitType = THROTTLE_LIMIT_TYPE_SCALE;
  config.output.throttleLimitPercent = 80;
  config.output.motorLimit = 90;
}

void setAcroTest(ModelConfig& config)
{
  setProfileName(config, name(PROFILE_ACRO_TEST));
  setCommonReceiver(config);
  setCommonModes(config);
  setCommonDshot300(config);

  config.input.rateType = RATES_TYPE_ACTUAL;
  config.input.rate[AXIS_ROLL] = 18;
  config.input.rate[AXIS_PITCH] = 18;
  config.input.rate[AXIS_YAW] = 16;
  config.input.superRate[AXIS_ROLL] = 65;
  config.input.superRate[AXIS_PITCH] = 65;
  config.input.superRate[AXIS_YAW] = 55;
  config.input.expo[AXIS_ROLL] = 20;
  config.input.expo[AXIS_PITCH] = 20;
  config.input.expo[AXIS_YAW] = 15;
  config.input.rateLimit[AXIS_ROLL] = 650;
  config.input.rateLimit[AXIS_PITCH] = 650;
  config.input.rateLimit[AXIS_YAW] = 550;

  config.pid[FC_PID_ROLL] = PidConfig{42, 85, 24, 72};
  config.pid[FC_PID_PITCH] = PidConfig{46, 90, 26, 76};
  config.pid[FC_PID_YAW] = PidConfig{45, 90, 0, 72};
  config.pid[FC_PID_LEVEL] = PidConfig{45, 0, 0, 0};

  config.level.angleLimit = 45;
  config.level.rateLimit = 300;
  config.dterm.filter = FilterConfig(FILTER_PT1, 128);
  config.dterm.filter2 = FilterConfig(FILTER_PT1, 128);
  config.yaw.filter = FilterConfig(FILTER_PT1, 90);
  config.controller.tpaScale = 10;
  config.controller.tpaBreakpoint = 1650;
  config.iterm.limit = 30;
  config.iterm.relaxCutoff = 15;
  config.arming.smallAngle = 25;

  config.output.throttleLimitType = THROTTLE_LIMIT_TYPE_SCALE;
  config.output.throttleLimitPercent = 95;
  config.output.motorLimit = 100;
}

} // namespace

const char* name(Profile profile)
{
  switch (profile)
  {
    case PROFILE_BENCH_SAFE: return "bench_safe";
    case PROFILE_HOVER_SAFE: return "hover_safe";
    case PROFILE_ACRO_TEST: return "acro_test";
    default: return "unknown";
  }
}

Profile parse(const char* value)
{
  if (!value) return PROFILE_UNKNOWN;
  if (std::strcmp(value, "bench_safe") == 0 || std::strcmp(value, "bench") == 0) return PROFILE_BENCH_SAFE;
  if (std::strcmp(value, "hover_safe") == 0 || std::strcmp(value, "hover") == 0) return PROFILE_HOVER_SAFE;
  if (std::strcmp(value, "acro_test") == 0 || std::strcmp(value, "acro") == 0) return PROFILE_ACRO_TEST;
  return PROFILE_UNKNOWN;
}

bool apply(ModelConfig& config, Profile profile)
{
  switch (profile)
  {
    case PROFILE_BENCH_SAFE:
      setBenchSafe(config);
      return true;
    case PROFILE_HOVER_SAFE:
      setHoverSafe(config);
      return true;
    case PROFILE_ACRO_TEST:
      setAcroTest(config);
      return true;
    default:
      return false;
  }
}

} // namespace Espfc::DroneProtoProfiles
