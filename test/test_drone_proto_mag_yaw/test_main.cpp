#include "Control/MagneticYawCorrection.hpp"

#include <cmath>
#include <iostream>

using Espfc::Control::MagneticYawCorrection;

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::cerr << "FAIL line " << __LINE__ << ": " #condition "\n"; \
    failures++; \
  } \
} while (false)

constexpr float PI_F = 3.14159265358979f;

bool near(float actual, float expected, float tolerance = 0.0001f)
{
  return fabsf(actual - expected) <= tolerance;
}

void learnReference(MagneticYawCorrection& correction, float field = 45.f)
{
  float yaw = 0.f;
  for (uint16_t i = 0; i < MagneticYawCorrection::REFERENCE_SAMPLES; i++)
  {
    correction.update(field, 30.f, 0.f, false, 0.f, yaw);
  }
}

void testBoschPolarityProducesNorthAndEastHeadings()
{
  CHECK(near(MagneticYawCorrection::bmm150Heading(-40.f, 0.f), 0.f));
  CHECK(near(MagneticYawCorrection::bmm150Heading(0.f, 40.f), PI_F * 0.5f));
}

void testReferenceLearningRequiresHealthyDisarmedSamples()
{
  MagneticYawCorrection correction;
  float yaw = 1.f;

  CHECK(!correction.update(5.f, 5.f, 0.f, false, 0.f, yaw));
  CHECK(!correction.update(45.f, 30.f, 0.f, true, 0.f, yaw));
  CHECK(!correction.referenceReady());

  learnReference(correction);
  CHECK(correction.referenceReady());
  CHECK(correction.initialized());
  CHECK(near(correction.referenceField(), 45.f));
}

void testCorrectionUsesShortestWrappedInnovation()
{
  MagneticYawCorrection correction;
  learnReference(correction);

  const float predicted = 179.f * PI_F / 180.f;
  const float heading = -179.f * PI_F / 180.f;
  float corrected = 0.f;
  CHECK(correction.update(45.f, 30.f, heading, true, predicted, corrected));
  CHECK(corrected > predicted);
  CHECK(near(corrected, 179.04f * PI_F / 180.f, 0.0002f));
}

void testInterferenceAndHeadingJumpsAreRejected()
{
  MagneticYawCorrection correction;
  learnReference(correction);

  float corrected = 0.f;
  CHECK(!correction.update(70.f, 30.f, 0.f, true, 0.f, corrected));
  CHECK(!correction.update(45.f, 30.f, 100.f * PI_F / 180.f, true, 0.f, corrected));
}

void testDisarmedReferenceTracksSlowlyAndArmedReferenceFreezes()
{
  MagneticYawCorrection correction;
  learnReference(correction);

  float corrected = 0.f;
  CHECK(correction.update(50.f, 30.f, 0.f, false, 0.f, corrected));
  const float disarmedReference = correction.referenceField();
  CHECK(disarmedReference > 45.f);

  CHECK(correction.update(50.f, 30.f, 0.f, true, 0.f, corrected));
  CHECK(near(correction.referenceField(), disarmedReference));
}

} // namespace

int main()
{
  testBoschPolarityProducesNorthAndEastHeadings();
  testReferenceLearningRequiresHealthyDisarmedSamples();
  testCorrectionUsesShortestWrappedInnovation();
  testInterferenceAndHeadingJumpsAreRejected();
  testDisarmedReferenceTracksSlowlyAndArmedReferenceFreezes();

  if (failures != 0)
  {
    std::cerr << failures << " magnetic yaw test(s) failed\n";
    return 1;
  }

  std::cout << "All DroneProto magnetic yaw tests passed\n";
  return 0;
}
