#include "../../lib/Espfc/src/Control/QrLocalization.cpp"

#include <cmath>
#include <cstring>
#include <iostream>

using namespace Espfc::Control;

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if(!(condition)) { \
    std::cerr << "FAIL line " << __LINE__ << ": " #condition "\n"; \
    failures++; \
  } \
} while(false)

bool near(float actual, float expected, float tolerance = 0.03f)
{
  return std::fabs(actual - expected) <= tolerance;
}

QrLocalizationInput centeredInput(uint16_t sequence, uint32_t nowMs)
{
  QrLocalizationInput input;
  input.sequence = sequence;
  input.nowMs = nowMs;
  input.geometry.valid = true;
  input.geometry.centerX = 0.5f;
  input.geometry.centerY = 0.5f;
  input.geometry.sideFraction = 0.1376f;
  input.geometry.areaFraction = 0.0189f;
  input.rangeValid = true;
  input.rangeM = 1.0f;
  return input;
}

void testLandmarkParserIsStrict()
{
  QrLandmark landmark;
  CHECK(QrLocalization::parseLandmark(
    "DLOC1,A1,1.000,2.000,0.000,90.0,0.200", landmark));
  CHECK(std::strcmp(landmark.id, "A1") == 0);
  CHECK(near(landmark.positionM[0], 1.0f));
  CHECK(near(landmark.positionM[1], 2.0f));
  CHECK(near(landmark.edgeYawDeg, 90.0f));
  CHECK(near(landmark.sizeM, 0.2f));

  CHECK(!QrLocalization::parseLandmark("https://example.com", landmark));
  CHECK(!QrLocalization::parseLandmark("DLOC1,,1,2,0,0,0.2", landmark));
  CHECK(!QrLocalization::parseLandmark("DLOC1,A 1,1,2,0,0,0.2", landmark));
  CHECK(!QrLocalization::parseLandmark("DLOC1,A1,1,2,0,0,0.01", landmark));
  CHECK(!QrLocalization::parseLandmark("DLOC1,A1,1,2,0,0,0.2,extra", landmark));
}

void testCenteredFloorMarkerInitializesWorldPosition()
{
  QrLocalizationConfig config;
  QrLocalizationState state;
  auto input = centeredInput(1, 1000);
  input.geometry.rotationRad = -1.57079632679f;

  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,FLOOR1,3.000,4.000,0.000,0.0,0.200", input));
  CHECK(state.valid);
  CHECK(QrLocalization::fresh(state, config, 1000));
  CHECK(near(state.positionWorldM[0], 3.0f));
  CHECK(near(state.positionWorldM[1], 4.0f));
  CHECK(near(state.positionWorldM[2], 1.0f, 0.05f));
  CHECK(state.headingValid);
  CHECK(near(state.headingWorldRad, 0.0f));
}

void testOffCenterFloorMarkerUsesCameraRayAndYaw()
{
  QrLocalizationConfig config;
  QrLocalizationState state;
  auto input = centeredInput(1, 1000);
  input.geometry.centerX = 0.60f;
  input.yawRad = 1.57079632679f;

  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,FLOOR2,0.000,0.000,0.000,0.0,0.200", input));
  CHECK(state.positionWorldM[0] > 0.10f);
  CHECK(near(state.positionWorldM[1], 0.0f, 0.04f));
}

void testForwardMarkerEstimatesPositionAlongCameraAxis()
{
  QrLocalizationConfig config;
  config.mount = QrCameraMount::FORWARD;
  QrLocalizationState state;
  auto input = centeredInput(1, 1000);
  input.rangeValid = false;

  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,WALL1,5.000,0.000,1.000,0.0,0.200", input));
  CHECK(near(state.positionWorldM[0], 4.0f, 0.05f));
  CHECK(near(state.positionWorldM[1], 0.0f));
  CHECK(near(state.positionWorldM[2], 1.0f));
  CHECK(!state.headingValid);
}

void testZbarFallbackDoesNotClaimHeading()
{
  QrLocalizationConfig config;
  QrLocalizationState state;
  auto input = centeredInput(1, 1000);
  input.geometry.zbarFallback = true;

  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,A1,0,0,0,0,0.2", input));
  CHECK(!state.headingValid);
}

void testFlowMotionAndResetPreserveWorldContinuity()
{
  QrLocalizationConfig config;
  QrLocalizationState state;
  auto input = centeredInput(1, 1000);
  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,A1,2.000,3.000,0.000,0.0,0.200", input));

  float local[3] = {0.5f, -0.25f, 0.1f};
  QrLocalization::update(state, config, local, 0, 1200);
  CHECK(near(state.positionWorldM[0], 2.5f));
  CHECK(near(state.positionWorldM[1], 2.75f));
  const float beforeReset[3] = {
    state.positionWorldM[0], state.positionWorldM[1], state.positionWorldM[2]
  };

  const float resetLocal[3] = {0.0f, 0.0f, 0.0f};
  QrLocalization::update(state, config, resetLocal, 1, 1300);
  CHECK(near(state.positionWorldM[0], beforeReset[0]));
  CHECK(near(state.positionWorldM[1], beforeReset[1]));
  CHECK(near(state.positionWorldM[2], beforeReset[2]));
}

void testTinyTiltedAndOrdinaryCodesAreRejected()
{
  QrLocalizationConfig config;
  QrLocalizationState state;
  auto input = centeredInput(1, 1000);
  input.geometry.sideFraction = 0.01f;
  CHECK(!QrLocalization::ingest(
    state, config, "DLOC1,A1,0,0,0,0,0.2", input));
  CHECK(state.lastReject == QrLocalizationReject::MARKER_TOO_SMALL);

  input = centeredInput(2, 1100);
  input.rollRad = 0.6f;
  CHECK(!QrLocalization::ingest(
    state, config, "DLOC1,A1,0,0,0,0,0.2", input));
  CHECK(state.lastReject == QrLocalizationReject::EXCESSIVE_TILT);

  input = centeredInput(3, 1200);
  CHECK(!QrLocalization::ingest(state, config, "ordinary QR text", input));
  CHECK(state.lastReject == QrLocalizationReject::NOT_LANDMARK);
}

void testLargeCorrectionRequiresTwoConsistentObservations()
{
  QrLocalizationConfig config;
  QrLocalizationState state;
  auto input = centeredInput(1, 1000);
  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,A1,0,0,0,0,0.2", input));

  input = centeredInput(2, 1200);
  CHECK(!QrLocalization::ingest(
    state, config, "DLOC1,B1,10,0,0,0,0.2", input));
  CHECK(state.lastReject == QrLocalizationReject::OUTLIER);
  CHECK(near(state.positionWorldM[0], 0.0f));

  input = centeredInput(3, 1400);
  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,B1,10,0,0,0,0.2", input));
  CHECK(state.lastReject == QrLocalizationReject::NONE);
  CHECK(state.positionWorldM[0] > 2.0f && state.positionWorldM[0] < 3.0f);
  CHECK(state.outlierRejectCount == 1);
}

void testDuplicateSequenceDoesNotMoveEstimate()
{
  QrLocalizationConfig config;
  QrLocalizationState state;
  auto input = centeredInput(7, 1000);
  CHECK(QrLocalization::ingest(
    state, config, "DLOC1,A1,1,1,0,0,0.2", input));
  const float x = state.positionWorldM[0];
  CHECK(!QrLocalization::ingest(
    state, config, "DLOC1,A1,2,2,0,0,0.2", input));
  CHECK(state.lastReject == QrLocalizationReject::DUPLICATE);
  CHECK(near(state.positionWorldM[0], x));
}

}

int main()
{
  testLandmarkParserIsStrict();
  testCenteredFloorMarkerInitializesWorldPosition();
  testOffCenterFloorMarkerUsesCameraRayAndYaw();
  testForwardMarkerEstimatesPositionAlongCameraAxis();
  testZbarFallbackDoesNotClaimHeading();
  testFlowMotionAndResetPreserveWorldContinuity();
  testTinyTiltedAndOrdinaryCodesAreRejected();
  testLargeCorrectionRequiresTwoConsistentObservations();
  testDuplicateSequenceDoesNotMoveEstimate();

  if(failures != 0)
  {
    std::cerr << failures << " QR localization test(s) failed\n";
    return 1;
  }
  std::cout << "All DroneProto QR localization tests passed\n";
  return 0;
}
