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
static constexpr uint16_t CONTINUOUS_STOP_US = 1500;
static constexpr uint16_t CONTINUOUS_RUN_US = 1600;
static constexpr uint16_t CONTINUOUS_STEP_DEGREES = 30;
static constexpr uint32_t CONTINUOUS_FULL_ROTATION_MS = 2400;
static constexpr uint32_t CONTINUOUS_STEP_INTERVAL_MS = 1000;
static constexpr uint32_t CONTINUOUS_STEP_RUN_MS = (CONTINUOUS_FULL_ROTATION_MS * CONTINUOUS_STEP_DEGREES) / 360;

static int8_t activePin = -1;
static uint16_t configuredMinUs = 1000;
static uint16_t configuredMaxUs = 2000;
static uint16_t configuredNeutralUs = 1500;
static uint16_t activeUs = 1500;
static bool continuousStepMode = false;
static bool continuousStepRunning = false;
static uint32_t continuousStepStartMs = 0;
static uint32_t continuousLastStepMs = 0;
static int continuousEstimatedAngleDeg = 0;

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
  ledcWrite(LEDC_CHANNEL, dutyFromUs(activeUs));
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
  continuousStepMode = false;
  continuousStepRunning = false;
  if (activePin < 0) return;

  ledcWrite(LEDC_CHANNEL, 0);
  ledcDetachPin(activePin);
  Hal::Gpio::pinMode(activePin, OUTPUT);
  Hal::Gpio::digitalWrite(activePin, LOW);
  activePin = -1;
}

void write(uint8_t pin, uint16_t us)
{
  continuousStepMode = false;
  continuousStepRunning = false;
  writeAttached(pin, us);
}

void writeDefault(uint16_t us)
{
  write(defaultPin(), us);
}

void startStepMode()
{
  if (!available()) return;

  continuousStepMode = true;
  continuousStepRunning = false;
  continuousLastStepMs = millis();
  writeAttached(defaultPin(), CONTINUOUS_STOP_US);
}

void stopStepMode()
{
  continuousStepMode = false;
  continuousStepRunning = false;
  writeAttached(defaultPin(), CONTINUOUS_STOP_US);
}

void update()
{
  if (!continuousStepMode) return;

  const uint32_t now = millis();
  if (continuousStepRunning)
  {
    if (now - continuousStepStartMs >= CONTINUOUS_STEP_RUN_MS)
    {
      writeAttached(defaultPin(), CONTINUOUS_STOP_US);
      continuousStepRunning = false;
      continuousLastStepMs = now;
    }
    return;
  }

  if (now - continuousLastStepMs >= CONTINUOUS_STEP_INTERVAL_MS)
  {
    writeAttached(defaultPin(), CONTINUOUS_RUN_US);
    continuousStepRunning = true;
    continuousStepStartMs = now;
    continuousEstimatedAngleDeg = (continuousEstimatedAngleDeg + CONTINUOUS_STEP_DEGREES) % 360;
  }
}

bool stepModeActive()
{
  return continuousStepMode;
}

bool stepRunning()
{
  return continuousStepRunning;
}

int estimatedAngleDeg()
{
  return continuousEstimatedAngleDeg;
}

uint32_t stepRunMs()
{
  return CONTINUOUS_STEP_RUN_MS;
}

#else

bool available() { return false; }
uint8_t defaultPin() { return 0; }
uint16_t minUs() { return 1000; }
uint16_t maxUs() { return 2000; }
uint16_t neutralUs() { return 1500; }
uint16_t currentUs() { return 1500; }
int8_t currentPin() { return -1; }
uint16_t clampUs(int us)
{
  if (us < 1000) return 1000;
  if (us > 2000) return 2000;
  return (uint16_t)us;
}
void setConfig(uint16_t, uint16_t, uint16_t) {}
void write(uint8_t, uint16_t) {}
void writeDefault(uint16_t) {}
void detach() {}
void startStepMode() {}
void stopStepMode() {}
void update() {}
bool stepModeActive() { return false; }
bool stepRunning() { return false; }
int estimatedAngleDeg() { return 0; }
uint32_t stepRunMs() { return 0; }

#endif

}

}

}
