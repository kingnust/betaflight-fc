#pragma once

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
int8_t currentPin();
uint16_t clampUs(int us);
void setConfig(uint16_t min, uint16_t max, uint16_t neutral);
void write(uint8_t pin, uint16_t us);
void writeDefault(uint16_t us);
void detach();
void startStepMode();
void stopStepMode();
void update();
bool stepModeActive();
bool stepRunning();
int estimatedAngleDeg();
uint32_t stepRunMs();

}

}

}
