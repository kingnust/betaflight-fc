#include "Device/DroneProtoServo.hpp"

#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_SERVO_PIN)
#include "Hal/Gpio.h"
#include <Arduino.h>
#endif

namespace Espfc {

namespace Device {

namespace DroneProtoServo {

#if defined(ESP32) && defined(ESPFC_DRONE_PROTO_SERVO_PIN)

static constexpr uint8_t LEDC_CHANNEL = 15;
static constexpr uint8_t RES_BITS = 16;
static constexpr uint32_t FREQ_HZ = 50;
static constexpr uint32_t PERIOD_US = 1000000UL / FREQ_HZ;
static constexpr uint16_t SAFE_MIN_US = 500;
static constexpr uint16_t SAFE_MAX_US = 2500;
static constexpr uint8_t POSITION_MIN_DEG = 0;
static constexpr uint8_t POSITION_CENTER_DEG = 90;
static constexpr uint8_t POSITION_MAX_DEG = 180;

static int8_t activePin = -1;
static uint16_t configuredMinUs = 1000;
static uint16_t configuredMaxUs = 2000;
static uint16_t configuredNeutralUs = 1500;
static uint16_t activeUs = 1500;
static uint8_t activeAngleDeg = POSITION_CENTER_DEG;

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
  writeAttached(pin, us);
}

void writeAngle(uint8_t pin, int angleDeg)
{
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

#else

bool available() { return false; }
uint8_t defaultPin() { return 0; }
uint16_t minUs() { return 1000; }
uint16_t maxUs() { return 2000; }
uint16_t neutralUs() { return 1500; }
uint16_t currentUs() { return 1500; }
uint8_t currentAngleDeg() { return 90; }
int8_t currentPin() { return -1; }
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
void write(uint8_t, uint16_t) {}
void writeAngle(uint8_t, int) {}
void writeDefault(uint16_t) {}
void writeDefaultAngle(int) {}
void centerDefault() {}
void detach() {}
void update() {}

#endif

}

}

}
