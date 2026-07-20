#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Espfc {

namespace Device {

namespace DroneProtoServo {

bool available();
uint8_t defaultPin();
uint16_t minUs();
uint16_t maxUs();
uint16_t neutralUs();
uint16_t currentUs();
uint8_t currentAngleDeg();
int8_t currentPin();
uint8_t forwardedChannel();
int8_t rate();
bool manualOverrideActive();
uint16_t clampUs(int us);
uint8_t clampAngleDeg(int angle);
uint16_t usFromAngleDeg(int angle);
void setConfig(uint16_t min, uint16_t max, uint16_t neutral);
void setForwarding(uint8_t channel, int8_t ratePercent);
void write(uint8_t pin, uint16_t us);
void writeAngle(uint8_t pin, int angleDeg);
void writeDefault(uint16_t us);
void writeDefaultAngle(int angleDeg);
void centerDefault();
void detach();
void update();
void updateInput(const float *inputUs, size_t channelCount, bool valid);

}

}

}
