#pragma once

#include <cmath>
#include <cstdint>

namespace Espfc::Control {

class ChirpExcitation
{
public:
  void begin(float startFrequencyHz, float endFrequencyHz, float durationSeconds, float sampleRateHz)
  {
    _startFrequencyHz = startFrequencyHz;
    _samplePeriod = sampleRateHz > 0.f ? 1.f / sampleRateHz : 0.f;
    _sampleCount = (_samplePeriod > 0.f && durationSeconds > 0.f)
      ? static_cast<uint32_t>(durationSeconds / _samplePeriod)
      : 0u;

    if(startFrequencyHz > 0.f && endFrequencyHz > startFrequencyHz && durationSeconds > 0.f && _sampleCount > 0u)
    {
      _beta = powf(endFrequencyHz / startFrequencyHz, 1.f / durationSeconds);
      _phaseScale = CHIRP_TWO_PI / logf(_beta);
      _phaseOffset = _phaseScale * startFrequencyHz;
      _valid = std::isfinite(_beta) && std::isfinite(_phaseScale);
    }
    else
    {
      _valid = false;
    }

    reset();
  }

  void reset()
  {
    _index = 0u;
    _finished = false;
    resetSignals();
  }

  bool update()
  {
    if(!_valid || _finished)
    {
      return false;
    }
    if(_index >= _sampleCount)
    {
      _finished = true;
      resetSignals();
      return false;
    }

    _frequencyHz = _startFrequencyHz * powf(_beta, static_cast<float>(_index) * _samplePeriod);
    _phase = fmodf(_phaseScale * _frequencyHz - _phaseOffset, CHIRP_TWO_PI);
    _excitation = cosf(_phase);

    // Keep the integrated angle excursion bounded at frequencies below 1 Hz.
    if(_frequencyHz < 1.f)
    {
      _excitation *= _frequencyHz;
    }

    _index++;
    return true;
  }

  bool valid() const { return _valid; }
  bool finished() const { return _finished; }
  float excitation() const { return _excitation; }
  float frequencyHz() const { return _frequencyHz; }
  float phase() const { return _phase; }
  uint32_t sampleCount() const { return _sampleCount; }

private:
  static constexpr float CHIRP_TWO_PI = 6.28318530717958647692f;

  void resetSignals()
  {
    _excitation = 0.f;
    _frequencyHz = 0.f;
    _phase = 0.f;
  }

  float _startFrequencyHz = 0.f;
  float _samplePeriod = 0.f;
  float _beta = 0.f;
  float _phaseScale = 0.f;
  float _phaseOffset = 0.f;
  float _excitation = 0.f;
  float _frequencyHz = 0.f;
  float _phase = 0.f;
  uint32_t _index = 0u;
  uint32_t _sampleCount = 0u;
  bool _valid = false;
  bool _finished = false;
};

} // namespace Espfc::Control
