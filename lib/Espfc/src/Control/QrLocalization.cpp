#include "Control/QrLocalization.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace Espfc::Control {

namespace {

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float MINIMUM_RANGE_M = 0.05f;

float clampUnit(float value)
{
  return std::clamp(value, 0.0f, 1.0f);
}

float normalizeAngle(float angle)
{
  while(angle > PI) angle -= 2.0f * PI;
  while(angle < -PI) angle += 2.0f * PI;
  return angle;
}

bool readFloat(const char *& cursor, float& value, bool finalField)
{
  if(!cursor || !*cursor) return false;
  char * end = nullptr;
  value = std::strtof(cursor, &end);
  if(end == cursor || !std::isfinite(value)) return false;
  if(finalField)
  {
    if(*end != 0) return false;
    cursor = end;
    return true;
  }
  if(*end != ',') return false;
  cursor = end + 1;
  return true;
}

bool validIdCharacter(char value)
{
  return (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
         (value >= '0' && value <= '9') || value == '_' || value == '-';
}

float distance3(const float first[3], const float second[3])
{
  const float x = first[0] - second[0];
  const float y = first[1] - second[1];
  const float z = first[2] - second[2];
  return std::sqrt(x * x + y * y + z * z);
}

void copyPosition(float destination[3], const float source[3])
{
  for(size_t i = 0; i < 3; i++) destination[i] = source[i];
}

void copyId(char destination[QrLandmark::ID_CAPACITY], const char * source)
{
  std::strncpy(destination, source ? source : "", QrLandmark::ID_CAPACITY - 1);
  destination[QrLandmark::ID_CAPACITY - 1] = 0;
}

bool reject(QrLocalizationState& state, QrLocalizationReject reason)
{
  state.lastReject = reason;
  state.rejectedCount++;
  switch(reason)
  {
    case QrLocalizationReject::NOT_LANDMARK: state.nonLandmarkCount++; break;
    case QrLocalizationReject::BAD_FORMAT: state.formatRejectCount++; break;
    case QrLocalizationReject::NO_GEOMETRY:
    case QrLocalizationReject::MARKER_TOO_SMALL:
      state.geometryRejectCount++;
      break;
    case QrLocalizationReject::EXCESSIVE_TILT: state.tiltRejectCount++; break;
    case QrLocalizationReject::RANGE_INVALID: state.rangeRejectCount++; break;
    case QrLocalizationReject::OUTLIER: state.outlierRejectCount++; break;
    case QrLocalizationReject::DUPLICATE: state.duplicateCount++; break;
    default: break;
  }
  return false;
}

}

bool QrLocalization::parseLandmark(const char * payload, QrLandmark& landmark)
{
  landmark = {};
  static constexpr char PREFIX[] = "DLOC1,";
  if(!payload || std::strncmp(payload, PREFIX, sizeof(PREFIX) - 1) != 0) return false;

  const char * cursor = payload + sizeof(PREFIX) - 1;
  const char * idEnd = std::strchr(cursor, ',');
  if(!idEnd) return false;
  const size_t idLength = static_cast<size_t>(idEnd - cursor);
  if(idLength == 0 || idLength >= QrLandmark::ID_CAPACITY) return false;
  for(size_t i = 0; i < idLength; i++)
  {
    if(!validIdCharacter(cursor[i])) return false;
    landmark.id[i] = cursor[i];
  }
  landmark.id[idLength] = 0;
  cursor = idEnd + 1;

  if(!readFloat(cursor, landmark.positionM[0], false) ||
     !readFloat(cursor, landmark.positionM[1], false) ||
     !readFloat(cursor, landmark.positionM[2], false) ||
     !readFloat(cursor, landmark.edgeYawDeg, false) ||
     !readFloat(cursor, landmark.sizeM, true)) return false;

  if(std::fabs(landmark.positionM[0]) > 1000.0f ||
     std::fabs(landmark.positionM[1]) > 1000.0f ||
     std::fabs(landmark.positionM[2]) > 100.0f ||
     std::fabs(landmark.edgeYawDeg) > 360.0f ||
     landmark.sizeM < 0.03f || landmark.sizeM > 2.0f) return false;
  return true;
}

void QrLocalization::update(QrLocalizationState& state,
  const QrLocalizationConfig& config, const float localPositionM[3],
  uint32_t localResetCount, uint32_t nowMs)
{
  if(!state.valid) return;
  if(localResetCount != state.lastLocalResetCount)
  {
    for(size_t i = 0; i < 3; i++)
    {
      state.mapOffsetM[i] = state.positionWorldM[i] - localPositionM[i];
    }
    state.lastLocalResetCount = localResetCount;
  }
  for(size_t i = 0; i < 3; i++)
  {
    state.positionWorldM[i] = localPositionM[i] + state.mapOffsetM[i];
  }

  const uint32_t age = nowMs - state.lastObservationAtMs;
  const float freshness = config.freshTimeMs == 0
    ? 0.0f
    : clampUnit(1.0f - static_cast<float>(age) / config.freshTimeMs);
  state.confidence = state.observationConfidence * freshness;
}

bool QrLocalization::ingest(QrLocalizationState& state,
  const QrLocalizationConfig& config, const char * payload,
  const QrLocalizationInput& input)
{
  if(!config.enabled) return reject(state, QrLocalizationReject::FEATURE_DISABLED);
  if(state.lastSequence == input.sequence && state.acceptedCount + state.rejectedCount != 0)
    return reject(state, QrLocalizationReject::DUPLICATE);
  state.lastSequence = input.sequence;

  if(!payload || std::strncmp(payload, "DLOC1,", 6) != 0)
    return reject(state, QrLocalizationReject::NOT_LANDMARK);
  QrLandmark landmark;
  if(!parseLandmark(payload, landmark))
    return reject(state, QrLocalizationReject::BAD_FORMAT);
  if(!input.geometry.valid)
    return reject(state, QrLocalizationReject::NO_GEOMETRY);
  if(input.geometry.centerX < 0.0f || input.geometry.centerX > 1.0f ||
     input.geometry.centerY < 0.0f || input.geometry.centerY > 1.0f ||
     input.geometry.sideFraction < config.minimumSideFraction ||
     input.geometry.sideFraction > 1.0f)
    return reject(state, QrLocalizationReject::MARKER_TOO_SMALL);
  if(std::max(std::fabs(input.rollRad), std::fabs(input.pitchRad)) > config.maximumTiltRad)
    return reject(state, QrLocalizationReject::EXCESSIVE_TILT);

  const float horizontalFov = config.horizontalFovDeg * DEG_TO_RAD;
  const float verticalFov = config.verticalFovDeg * DEG_TO_RAD;
  if(horizontalFov <= 0.1f || horizontalFov >= PI - 0.1f ||
     verticalFov <= 0.1f || verticalFov >= PI - 0.1f)
    return reject(state, QrLocalizationReject::RANGE_INVALID);

  const float focalFraction = 0.5f / std::tan(horizontalFov * 0.5f);
  const float qrRange = landmark.sizeM * focalFraction / input.geometry.sideFraction;
  if(!std::isfinite(qrRange) || qrRange < MINIMUM_RANGE_M || qrRange > config.maximumRangeM)
    return reject(state, QrLocalizationReject::RANGE_INVALID);

  float range = qrRange;
  float rangeDisagreement = 0.0f;
  const bool externalRangeUsed = config.mount == QrCameraMount::DOWNWARD && input.rangeValid;
  if(externalRangeUsed)
  {
    if(input.rangeM < MINIMUM_RANGE_M || input.rangeM > config.maximumRangeM)
      return reject(state, QrLocalizationReject::RANGE_INVALID);
    rangeDisagreement = std::fabs(input.rangeM - qrRange) /
      std::max(input.rangeM, qrRange);
    if(rangeDisagreement > config.maximumRangeDisagreement)
      return reject(state, QrLocalizationReject::RANGE_INVALID);
    range = input.rangeM * 0.75f + qrRange * 0.25f;
  }

  const float rayRight = (input.geometry.centerX - 0.5f) *
    2.0f * std::tan(horizontalFov * 0.5f);
  const float rayDown = (input.geometry.centerY - 0.5f) *
    2.0f * std::tan(verticalFov * 0.5f);
  float markerForward = 0.0f;
  float markerRight = 0.0f;
  float observedWorld[3] = {};
  if(config.mount == QrCameraMount::DOWNWARD)
  {
    markerForward = -rayDown * range;
    markerRight = rayRight * range;
    observedWorld[2] = landmark.positionM[2] + range;
  }
  else
  {
    markerForward = range;
    markerRight = rayRight * range;
    observedWorld[2] = landmark.positionM[2] + rayDown * range;
  }

  const float cosine = std::cos(input.yawRad);
  const float sine = std::sin(input.yawRad);
  const float markerEarthX = cosine * markerForward - sine * markerRight;
  const float markerEarthY = sine * markerForward + cosine * markerRight;
  observedWorld[0] = landmark.positionM[0] - markerEarthX;
  observedWorld[1] = landmark.positionM[1] - markerEarthY;

  update(state, config, input.localPositionM, input.localResetCount, input.nowMs);
  const bool hadValidEstimate = state.valid;
  float innovation[3] = {};
  for(size_t i = 0; i < 3; i++) innovation[i] = observedWorld[i] - state.positionWorldM[i];
  const float horizontalInnovation = std::hypot(innovation[0], innovation[1]);
  const bool largeInnovation = hadValidEstimate &&
    (horizontalInnovation > config.maximumInnovationM ||
     std::fabs(innovation[2]) > config.maximumVerticalInnovationM);

  bool confirmedOutlier = false;
  if(largeInnovation)
  {
    confirmedOutlier = state.pendingOutlier &&
      std::strcmp(state.pendingLandmarkId, landmark.id) == 0 &&
      input.nowMs - state.pendingOutlierAtMs <= config.outlierConfirmationMs &&
      distance3(state.pendingWorldM, observedWorld) <= config.outlierConfirmationM;
    if(!confirmedOutlier)
    {
      state.pendingOutlier = true;
      state.pendingOutlierAtMs = input.nowMs;
      copyId(state.pendingLandmarkId, landmark.id);
      copyPosition(state.pendingWorldM, observedWorld);
      state.innovationM = distance3(observedWorld, state.positionWorldM);
      return reject(state, QrLocalizationReject::OUTLIER);
    }
  }

  const float centerRadius = std::hypot(
    (input.geometry.centerX - 0.5f) * 2.0f,
    (input.geometry.centerY - 0.5f) * 2.0f);
  const float centerQuality = clampUnit(1.0f - centerRadius / 1.2f);
  const float sizeQuality = clampUnit(
    (input.geometry.sideFraction - config.minimumSideFraction) /
    std::max(0.001f, 0.20f - config.minimumSideFraction));
  const float rangeQuality = externalRangeUsed
    ? clampUnit(1.0f - rangeDisagreement /
        std::max(0.001f, config.maximumRangeDisagreement))
    : 0.55f;
  const float quality = clampUnit(
    centerQuality * 0.35f + sizeQuality * 0.45f + rangeQuality * 0.20f);
  float gain = config.minimumCorrectionGain +
    (config.maximumCorrectionGain - config.minimumCorrectionGain) * quality;
  if(confirmedOutlier) gain = std::min(gain, 0.25f);

  if(!hadValidEstimate)
  {
    for(size_t i = 0; i < 3; i++) state.mapOffsetM[i] = observedWorld[i] - input.localPositionM[i];
    state.valid = true;
  }
  else
  {
    for(size_t i = 0; i < 3; i++) state.mapOffsetM[i] += innovation[i] * gain;
  }

  state.lastLocalResetCount = input.localResetCount;
  state.lastObservationAtMs = input.nowMs;
  copyId(state.lastLandmarkId, landmark.id);
  copyPosition(state.observedWorldM, observedWorld);
  state.cameraRangeM = range;
  state.rangeDisagreement = rangeDisagreement;
  state.innovationM = hadValidEstimate
    ? distance3(observedWorld, state.positionWorldM)
    : 0.0f;
  state.observationConfidence = 0.35f + 0.65f * quality;
  state.headingValid = config.mount == QrCameraMount::DOWNWARD &&
    !input.geometry.mirrored && !input.geometry.zbarFallback;
  if(state.headingValid)
  {
    state.headingWorldRad = normalizeAngle(
      landmark.edgeYawDeg * DEG_TO_RAD - (input.geometry.rotationRad + PI * 0.5f));
  }
  state.pendingOutlier = false;
  state.pendingLandmarkId[0] = 0;
  state.lastReject = QrLocalizationReject::NONE;
  state.acceptedCount++;
  update(state, config, input.localPositionM, input.localResetCount, input.nowMs);
  return true;
}

void QrLocalization::reset(QrLocalizationState& state)
{
  state = {};
}

bool QrLocalization::fresh(const QrLocalizationState& state,
  const QrLocalizationConfig& config, uint32_t nowMs)
{
  return state.valid && state.lastObservationAtMs != 0 &&
    nowMs - state.lastObservationAtMs <= config.freshTimeMs;
}

const char * QrLocalization::rejectName(QrLocalizationReject reject)
{
  switch(reject)
  {
    case QrLocalizationReject::NONE: return "NONE";
    case QrLocalizationReject::FEATURE_DISABLED: return "DISABLED";
    case QrLocalizationReject::NOT_LANDMARK: return "NOT_LANDMARK";
    case QrLocalizationReject::BAD_FORMAT: return "BAD_FORMAT";
    case QrLocalizationReject::NO_GEOMETRY: return "NO_GEOMETRY";
    case QrLocalizationReject::MARKER_TOO_SMALL: return "MARKER_TOO_SMALL";
    case QrLocalizationReject::EXCESSIVE_TILT: return "EXCESSIVE_TILT";
    case QrLocalizationReject::RANGE_INVALID: return "RANGE_INVALID";
    case QrLocalizationReject::OUTLIER: return "OUTLIER";
    case QrLocalizationReject::DUPLICATE: return "DUPLICATE";
    default: return "UNKNOWN";
  }
}

const char * QrLocalization::mountName(QrCameraMount mount)
{
  return mount == QrCameraMount::FORWARD ? "FORWARD" : "DOWNWARD";
}

}
