#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Control {

enum class QrCameraMount : uint8_t
{
  DOWNWARD = 0,
  FORWARD = 1,
};

enum class QrLocalizationReject : uint8_t
{
  NONE = 0,
  FEATURE_DISABLED,
  NOT_LANDMARK,
  BAD_FORMAT,
  NO_GEOMETRY,
  MARKER_TOO_SMALL,
  EXCESSIVE_TILT,
  RANGE_INVALID,
  OUTLIER,
  DUPLICATE,
};

struct QrLandmark
{
  static constexpr size_t ID_CAPACITY = 16;
  char id[ID_CAPACITY] = {};
  float positionM[3] = {0.0f, 0.0f, 0.0f};
  float edgeYawDeg = 0.0f;
  float sizeM = 0.0f;
};

struct QrImageGeometry
{
  bool valid = false;
  bool mirrored = false;
  bool zbarFallback = false;
  float centerX = 0.5f;
  float centerY = 0.5f;
  float sideFraction = 0.0f;
  float areaFraction = 0.0f;
  float rotationRad = 0.0f;
};

struct QrLocalizationConfig
{
  bool enabled = true;
  QrCameraMount mount = QrCameraMount::DOWNWARD;
  float horizontalFovDeg = 72.0f;
  float verticalFovDeg = 55.0f;
  float minimumSideFraction = 0.025f;
  float maximumRangeM = 8.0f;
  float maximumTiltRad = 0.436332f;
  float maximumRangeDisagreement = 0.55f;
  float maximumInnovationM = 1.5f;
  float maximumVerticalInnovationM = 1.0f;
  float outlierConfirmationM = 0.35f;
  uint32_t outlierConfirmationMs = 1000;
  uint32_t freshTimeMs = 2500;
  float minimumCorrectionGain = 0.15f;
  float maximumCorrectionGain = 0.55f;
};

struct QrLocalizationInput
{
  uint16_t sequence = 0;
  uint32_t nowMs = 0;
  QrImageGeometry geometry;
  float localPositionM[3] = {0.0f, 0.0f, 0.0f};
  uint32_t localResetCount = 0;
  float yawRad = 0.0f;
  float rollRad = 0.0f;
  float pitchRad = 0.0f;
  bool rangeValid = false;
  float rangeM = 0.0f;
};

struct QrLocalizationState
{
  bool valid = false;
  bool headingValid = false;
  uint16_t lastSequence = 0;
  uint32_t lastObservationAtMs = 0;
  uint32_t lastLocalResetCount = 0;
  char lastLandmarkId[QrLandmark::ID_CAPACITY] = {};
  float mapOffsetM[3] = {0.0f, 0.0f, 0.0f};
  float positionWorldM[3] = {0.0f, 0.0f, 0.0f};
  float observedWorldM[3] = {0.0f, 0.0f, 0.0f};
  float headingWorldRad = 0.0f;
  float cameraRangeM = 0.0f;
  float rangeDisagreement = 0.0f;
  float innovationM = 0.0f;
  float observationConfidence = 0.0f;
  float confidence = 0.0f;
  bool pendingOutlier = false;
  uint32_t pendingOutlierAtMs = 0;
  char pendingLandmarkId[QrLandmark::ID_CAPACITY] = {};
  float pendingWorldM[3] = {0.0f, 0.0f, 0.0f};
  QrLocalizationReject lastReject = QrLocalizationReject::NONE;
  uint32_t acceptedCount = 0;
  uint32_t rejectedCount = 0;
  uint32_t duplicateCount = 0;
  uint32_t nonLandmarkCount = 0;
  uint32_t formatRejectCount = 0;
  uint32_t geometryRejectCount = 0;
  uint32_t tiltRejectCount = 0;
  uint32_t rangeRejectCount = 0;
  uint32_t outlierRejectCount = 0;
};

class QrLocalization
{
  public:
    static bool parseLandmark(const char * payload, QrLandmark& landmark);
    static bool ingest(QrLocalizationState& state, const QrLocalizationConfig& config,
      const char * payload, const QrLocalizationInput& input);
    static void update(QrLocalizationState& state, const QrLocalizationConfig& config,
      const float localPositionM[3], uint32_t localResetCount, uint32_t nowMs);
    static void reset(QrLocalizationState& state);
    static bool fresh(const QrLocalizationState& state,
      const QrLocalizationConfig& config, uint32_t nowMs);
    static const char * rejectName(QrLocalizationReject reject);
    static const char * mountName(QrCameraMount mount);
};

}
