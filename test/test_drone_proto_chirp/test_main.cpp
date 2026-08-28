#include "Control/ChirpExcitation.hpp"

#include <cmath>
#include <iostream>

using Espfc::Control::ChirpExcitation;

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if(!(condition)) { \
    std::cerr << "FAIL line " << __LINE__ << ": " #condition "\n"; \
    failures++; \
  } \
} while(false)

bool near(float actual, float expected, float tolerance = 0.001f)
{
  return fabsf(actual - expected) <= tolerance;
}

void testExponentialSweepShape()
{
  ChirpExcitation chirp;
  chirp.begin(0.5f, 100.f, 20.f, 500.f);

  CHECK(chirp.valid());
  CHECK(chirp.sampleCount() == 10000u);
  CHECK(chirp.update());
  CHECK(near(chirp.frequencyHz(), 0.5f));
  CHECK(near(chirp.phase(), 0.f));
  CHECK(near(chirp.excitation(), 0.5f));

  float previousFrequency = chirp.frequencyHz();
  for(uint32_t i = 1; i < chirp.sampleCount(); i++)
  {
    CHECK(chirp.update());
    CHECK(chirp.frequencyHz() >= previousFrequency);
    CHECK(fabsf(chirp.excitation()) <= 1.0001f);
    previousFrequency = chirp.frequencyHz();
  }
  CHECK(previousFrequency > 99.f && previousFrequency <= 100.f);
  CHECK(!chirp.update());
  CHECK(chirp.finished());
  CHECK(near(chirp.frequencyHz(), 0.f));
  CHECK(near(chirp.excitation(), 0.f));
}

void testResetAndInvalidConfiguration()
{
  ChirpExcitation chirp;
  chirp.begin(1.f, 10.f, 1.f, 100.f);
  CHECK(chirp.update());
  chirp.reset();
  CHECK(!chirp.finished());
  CHECK(chirp.update());
  CHECK(near(chirp.frequencyHz(), 1.f));

  chirp.begin(10.f, 1.f, 1.f, 100.f);
  CHECK(!chirp.valid());
  CHECK(!chirp.update());
}

} // namespace

int main()
{
  testExponentialSweepShape();
  testResetAndInvalidConfiguration();

  if(failures != 0)
  {
    std::cerr << failures << " chirp test(s) failed\n";
    return 1;
  }

  std::cout << "All DroneProto chirp tests passed\n";
  return 0;
}
