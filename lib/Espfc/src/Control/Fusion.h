#pragma once

#include "Model.h"
#include <Madgwick.h>
#include <Mahony.h>
#if defined(ESPFC_DRONE_PROTO_MAG_YAW)
#include "Control/MagneticYawCorrection.hpp"
#endif

namespace Espfc {

namespace Control {

class Fusion
{
  public:
    Fusion(Model& model);
    int begin();
    void restoreGain();
    int update();

    void experimentalFusion();
    void simpleFusion();
    void kalmanFusion();
    void complementaryFusion();
    void complementaryFusionOld();
    void rtqfFusion();
    void updatePoseFromAccelMag();

    // experimental
    void lerpFusion();
    void madgwickFusion();
    void mahonyFusion();

  private:
#if defined(ESPFC_DRONE_PROTO_MAG_YAW)
    float correctYawWithMag(float predictedYaw);
#endif

    Model& _model;
    bool _first;
    Madgwick _madgwick;
    Mahony _mahony;
#if defined(ESPFC_DRONE_PROTO_MAG_YAW)
    uint32_t _magSampleCount = 0;
    MagneticYawCorrection _magYawCorrection;
#endif
};

}

}
