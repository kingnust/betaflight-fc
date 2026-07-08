#pragma once

#include "ModelConfig.h"

namespace Espfc::DroneProtoProfiles {

enum Profile
{
  PROFILE_BENCH_SAFE,
  PROFILE_HOVER_SAFE,
  PROFILE_ACRO_TEST,
  PROFILE_UNKNOWN,
};

const char* name(Profile profile);
Profile parse(const char* value);
bool apply(ModelConfig& config, Profile profile);

} // namespace Espfc::DroneProtoProfiles
