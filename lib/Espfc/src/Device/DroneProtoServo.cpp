#include "Device/DroneProtoServo.hpp"

#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_SERVO_PIN)
#include "Hal/Gpio.h"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#endif

namespace Espfc {

namespace Device {

namespace DroneProtoServo {

#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_SERVO_PIN)

static constexpr uint8_t LEDC_CHANNEL = 7;
static constexpr uint8_t RES_BITS = 14;
static constexpr uint32_t FREQ_HZ = 50;
static constexpr uint32_t PERIOD_US = 1000000UL / FREQ_HZ;
static constexpr uint16_t SAFE_MIN_US = 500;
static constexpr uint16_t SAFE_MAX_US = 2500;
static constexpr uint8_t POSITION_MIN_DEG = 0;
static constexpr uint8_t POSITION_CENTER_DEG = 90;
static constexpr uint8_t POSITION_MAX_DEG = 180;
static constexpr uint8_t CHANNEL_FORWARDING_DISABLED = 0xff;
static constexpr uint8_t DEFAULT_FORWARD_CHANNEL = 7;  // RC CH8, RadioMaster rear roller.
static constexpr uint32_t MANUAL_OVERRIDE_MS = 750;
static constexpr uint32_t INPUT_UPDATE_INTERVAL_MS = 20;
static constexpr uint32_t INVALID_INPUT_HOLD_MS = 500;
static constexpr float INPUT_FILTER_ALPHA = 0.25f;
static constexpr float INPUT_HYSTERESIS_US = 6.0f;
static constexpr float INPUT_CENTER_DEADBAND_US = 8.0f;
static constexpr uint16_t OUTPUT_QUANTUM_US = 4;

static int8_t activePin = -1;
static uint16_t configuredMinUs = 1000;
static uint16_t configuredMaxUs = 2000;
static uint16_t configuredNeutralUs = 1500;
static uint16_t activeUs = 1500;
static uint8_t activeAngleDeg = POSITION_CENTER_DEG;
static uint8_t configuredForwardedChannel = DEFAULT_FORWARD_CHANNEL;
static int8_t configuredRate = 100;
static uint32_t manualOverrideUntilMs = 0;
static uint32_t nextInputUpdateMs = 0;
static uint32_t lastValidInputMs = 0;
static float filteredInputUs = 1500.0f;
static bool inputFilterInitialized = false;

static uint8_t angleFromUs(uint16_t us);

static uint32_t dutyFromUs(uint16_t us)
{
  const uint32_t maxDuty = (1UL << RES_BITS) - 1UL;
  return ((uint32_t)us * maxDuty + (PERIOD_US / 2)) / PERIOD_US;
}

static void writeAttached(uint8_t pin, uint16_t us)
{
  if (!available()) return;

  if (activePin != -1 && activePin != (int8_t)pin)
  {
    detach();
  }

  if (activePin != (int8_t)pin)
  {
    ledcSetup(LEDC_CHANNEL, FREQ_HZ, RES_BITS);
    ledcAttachPin(pin, LEDC_CHANNEL);
    activePin = (int8_t)pin;
  }

  activeUs = clampUs(us);
  activeAngleDeg = angleFromUs(activeUs);
  ledcWrite(LEDC_CHANNEL, dutyFromUs(activeUs));
}

static uint16_t quantizeUs(uint16_t us)
{
  const uint16_t quantized = ((us + (OUTPUT_QUANTUM_US / 2)) / OUTPUT_QUANTUM_US) * OUTPUT_QUANTUM_US;
  return clampUs(quantized);
}

static uint8_t mapUsToAngle(uint16_t us, uint16_t inMin, uint16_t inMax, uint8_t outMin, uint8_t outMax)
{
  if (inMax <= inMin) return outMin;
  if (us <= inMin) return outMin;
  if (us >= inMax) return outMax;
  const uint32_t numerator = (uint32_t)(us - inMin) * (outMax - outMin) + ((inMax - inMin) / 2);
  return outMin + (uint8_t)(numerator / (inMax - inMin));
}

static uint8_t angleFromUs(uint16_t us)
{
  if (us <= configuredNeutralUs)
  {
    return mapUsToAngle(us, configuredMinUs, configuredNeutralUs, POSITION_MIN_DEG, POSITION_CENTER_DEG);
  }
  return mapUsToAngle(us, configuredNeutralUs, configuredMaxUs, POSITION_CENTER_DEG, POSITION_MAX_DEG);
}

bool available()
{
  return ESPFC_DRONE_PROTO_SERVO_PIN >= 0;
}

uint8_t defaultPin()
{
  return (uint8_t)ESPFC_DRONE_PROTO_SERVO_PIN;
}

uint16_t minUs()
{
  return configuredMinUs;
}

uint16_t maxUs()
{
  return configuredMaxUs;
}

uint16_t neutralUs()
{
  return configuredNeutralUs;
}

uint16_t currentUs()
{
  return activeUs;
}

uint8_t currentAngleDeg()
{
  return activeAngleDeg;
}

int8_t currentPin()
{
  return activePin;
}

uint8_t forwardedChannel()
{
  return configuredForwardedChannel;
}

int8_t rate()
{
  return configuredRate;
}

bool manualOverrideActive()
{
  return static_cast<int32_t>(millis() - manualOverrideUntilMs) < 0;
}

uint16_t clampUs(int us)
{
  if (us < configuredMinUs) return configuredMinUs;
  if (us > configuredMaxUs) return configuredMaxUs;
  return (uint16_t)us;
}

uint8_t clampAngleDeg(int angle)
{
  if (angle < POSITION_MIN_DEG) return POSITION_MIN_DEG;
  if (angle > POSITION_MAX_DEG) return POSITION_MAX_DEG;
  return (uint8_t)angle;
}

uint16_t usFromAngleDeg(int angle)
{
  const uint8_t clamped = clampAngleDeg(angle);
  if (clamped <= POSITION_CENTER_DEG)
  {
    const uint32_t numerator = (uint32_t)(configuredNeutralUs - configuredMinUs) * clamped + (POSITION_CENTER_DEG / 2);
    return configuredMinUs + (uint16_t)(numerator / POSITION_CENTER_DEG);
  }

  const uint8_t upperAngle = clamped - POSITION_CENTER_DEG;
  const uint32_t numerator = (uint32_t)(configuredMaxUs - configuredNeutralUs) * upperAngle + (POSITION_CENTER_DEG / 2);
  return configuredNeutralUs + (uint16_t)(numerator / POSITION_CENTER_DEG);
}

void setConfig(uint16_t min, uint16_t max, uint16_t neutral)
{
  if (min < SAFE_MIN_US) min = SAFE_MIN_US;
  if (max > SAFE_MAX_US) max = SAFE_MAX_US;
  if (max <= min)
  {
    min = 1000;
    max = 2000;
  }
  configuredMinUs = min;
  configuredMaxUs = max;
  configuredNeutralUs = clampUs(neutral);
}

void setForwarding(uint8_t channel, int8_t ratePercent)
{
  configuredForwardedChannel = channel;
  configuredRate = std::clamp((int)ratePercent, -100, 100);
}

void detach()
{
  if (activePin < 0) return;

  ledcWrite(LEDC_CHANNEL, 0);
  ledcDetachPin(activePin);
  Hal::Gpio::pinMode(activePin, OUTPUT);
  Hal::Gpio::digitalWrite(activePin, LOW);
  activePin = -1;
}

void write(uint8_t pin, uint16_t us)
{
  manualOverrideUntilMs = millis() + MANUAL_OVERRIDE_MS;
  writeAttached(pin, us);
}

void writeAngle(uint8_t pin, int angleDeg)
{
  manualOverrideUntilMs = millis() + MANUAL_OVERRIDE_MS;
  activeAngleDeg = clampAngleDeg(angleDeg);
  writeAttached(pin, usFromAngleDeg(activeAngleDeg));
}

void writeDefault(uint16_t us)
{
  write(defaultPin(), us);
}

void writeDefaultAngle(int angleDeg)
{
  writeAngle(defaultPin(), angleDeg);
}

void centerDefault()
{
  writeDefaultAngle(POSITION_CENTER_DEG);
}

void update()
{
}

void updateInput(const float *inputUs, size_t channelCount, bool valid)
{
  if(!available() || configuredForwardedChannel == CHANNEL_FORWARDING_DISABLED || manualOverrideActive()) return;

  const uint32_t nowMs = millis();

  if(!valid || inputUs == nullptr || configuredForwardedChannel >= channelCount)
  {
    if(lastValidInputMs != 0 && nowMs - lastValidInputMs <= INVALID_INPUT_HOLD_MS) return;
    inputFilterInitialized = false;
    if(activeUs != configuredNeutralUs) writeAttached(defaultPin(), configuredNeutralUs);
    return;
  }

  lastValidInputMs = nowMs;
  if(static_cast<int32_t>(nowMs - nextInputUpdateMs) < 0) return;
  nextInputUpdateMs = nowMs + INPUT_UPDATE_INTERVAL_MS;

  float input = std::clamp(inputUs[configuredForwardedChannel], 1000.0f, 2000.0f);
  if(std::fabs(input - 1500.0f) <= INPUT_CENTER_DEADBAND_US) input = 1500.0f;
  if(!inputFilterInitialized)
  {
    filteredInputUs = input;
    inputFilterInitialized = true;
  }
  else if(std::fabs(input - filteredInputUs) > INPUT_HYSTERESIS_US)
  {
    filteredInputUs += (input - filteredInputUs) * INPUT_FILTER_ALPHA;
  }

  float normalized = (filteredInputUs - 1500.0f) / 500.0f;
  normalized *= configuredRate * 0.01f;

  float targetUs = configuredNeutralUs;
  if(normalized < 0.0f)
  {
    targetUs += normalized * (configuredNeutralUs - configuredMinUs);
  }
  else
  {
    targetUs += normalized * (configuredMaxUs - configuredNeutralUs);
  }

  const uint16_t target = quantizeUs(clampUs(lrintf(targetUs)));
  if(target != activeUs) writeAttached(defaultPin(), target);
}

#else

bool available() { return false; }
uint8_t defaultPin() { return 0; }
uint16_t minUs() { return 1000; }
uint16_t maxUs() { return 2000; }
uint16_t neutralUs() { return 1500; }
uint16_t currentUs() { return 1500; }
uint8_t currentAngleDeg() { return 90; }
int8_t currentPin() { return -1; }
uint8_t forwardedChannel() { return 0xff; }
int8_t rate() { return 100; }
bool manualOverrideActive() { return false; }
uint16_t clampUs(int us)
{
  if (us < 1000) return 1000;
  if (us > 2000) return 2000;
  return (uint16_t)us;
}
uint8_t clampAngleDeg(int angle)
{
  if (angle < 0) return 0;
  if (angle > 180) return 180;
  return (uint8_t)angle;
}
uint16_t usFromAngleDeg(int angle)
{
  return 1000 + ((uint32_t)clampAngleDeg(angle) * 1000 + 90) / 180;
}
void setConfig(uint16_t, uint16_t, uint16_t) {}
void setForwarding(uint8_t, int8_t) {}
void write(uint8_t, uint16_t) {}
void writeAngle(uint8_t, int) {}
void writeDefault(uint16_t) {}
void writeDefaultAngle(int) {}
void centerDefault() {}
void detach() {}
void update() {}
void updateInput(const float *, size_t, bool) {}

#endif

}

}

}
