#pragma once

#include <cmath>
#include <cstdint>

namespace Espfc::Control {

class MagneticYawCorrection
{
public:
  static constexpr uint16_t REFERENCE_SAMPLES = 15;

  void reset()
  {
    _referenceSum = 0.f;
    _referenceField = 0.f;
    _referenceSamples = 0;
    _referenceReady = false;
    _initialized = false;
  }

  static float wrapAngle(float angle)
  {
    constexpr float PI_F = 3.14159265358979f;
    constexpr float TWO_PI_F = PI_F * 2.f;
    while (angle > PI_F) angle -= TWO_PI_F;
    while (angle < -PI_F) angle += TWO_PI_F;
    return angle;
  }

  static float bmm150Heading(float leveledFieldX, float leveledFieldY)
  {
    // Bosch defines the measured field opposite the north-seeking vector.
    return wrapAngle(-atan2f(-leveledFieldY, -leveledFieldX));
  }

  bool update(
    float fieldStrength,
    float horizontalFieldStrength,
    float magneticHeading,
    bool armed,
    float predictedYaw,
    float& correctedYaw)
  {
    if (!validField(fieldStrength, horizontalFieldStrength) || !std::isfinite(magneticHeading)) return false;

    if (!_referenceReady)
    {
      if (armed) return false;
      _referenceSum += fieldStrength;
      _referenceSamples++;
      if (_referenceSamples < REFERENCE_SAMPLES) return false;

      _referenceField = _referenceSum / static_cast<float>(_referenceSamples);
      _referenceReady = true;
      _initialized = true;
      correctedYaw = wrapAngle(magneticHeading);
      return true;
    }

    if (fieldStrength < _referenceField * 0.65f || fieldStrength > _referenceField * 1.35f) return false;

    const float innovation = wrapAngle(magneticHeading - predictedYaw);
    constexpr float MAX_INNOVATION = 1.0471975512f; // 60 degrees
    if (fabsf(innovation) > MAX_INNOVATION) return false;

    if (!armed)
    {
      constexpr float REFERENCE_GAIN = 0.01f;
      _referenceField += (fieldStrength - _referenceField) * REFERENCE_GAIN;
    }

    constexpr float CORRECTION_GAIN = 0.02f;
    correctedYaw = wrapAngle(predictedYaw + innovation * CORRECTION_GAIN);
    _initialized = true;
    return true;
  }

  bool referenceReady() const { return _referenceReady; }
  bool initialized() const { return _initialized; }
  float referenceField() const { return _referenceField; }

private:
  static bool validField(float fieldStrength, float horizontalFieldStrength)
  {
    // Bosch's compensated integer output is in microtesla. This broad range
    // covers Earth's field while rejecting zero, overflow, and strong local fields.
    return std::isfinite(fieldStrength)
      && std::isfinite(horizontalFieldStrength)
      && fieldStrength >= 10.f
      && fieldStrength <= 100.f
      && horizontalFieldStrength >= 5.f
      && horizontalFieldStrength <= fieldStrength + 0.5f;
  }

  float _referenceSum = 0.f;
  float _referenceField = 0.f;
  uint16_t _referenceSamples = 0;
  bool _referenceReady = false;
  bool _initialized = false;
};

} // namespace Espfc::Control
