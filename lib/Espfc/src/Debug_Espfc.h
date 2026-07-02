#ifndef _ESPFC_DEBUG_H_
#define _ESPFC_DEBUG_H_

#include <Arduino.h>

#ifdef ESPFC_DRONE_PROTO_ACTIVE_DEBUG
#define DRONE_PROTO_DEBUG_LINE(v) do { Serial.print("ACTIVE BMI088 DEBUG: "); Serial.println(v); Serial.flush(); } while(0)
#define DRONE_PROTO_DEBUG_VALUE(k, v) do { Serial.print("ACTIVE BMI088 DEBUG: "); Serial.print(k); Serial.print("="); Serial.println(v); Serial.flush(); } while(0)
#define DRONE_PROTO_DEBUG_HEX(k, v) do { Serial.print("ACTIVE BMI088 DEBUG: "); Serial.print(k); Serial.print("=0x"); Serial.println(v, HEX); Serial.flush(); } while(0)
#else
#define DRONE_PROTO_DEBUG_LINE(v)
#define DRONE_PROTO_DEBUG_VALUE(k, v)
#define DRONE_PROTO_DEBUG_HEX(k, v)
#endif

#ifdef ESPFC_DEBUG_PIN
#include "Hal/Gpio.h"
#define PIN_DEBUG(v) ::Espfc::Hal::Gpio::digitalWrite(ESPFC_DEBUG_PIN, v)
#define PIN_DEBUG_INIT() ::Espfc::Hal::Gpio::pinMode(ESPFC_DEBUG_PIN, OUTPUT)
#else
#define PIN_DEBUG(v)
#define PIN_DEBUG_INIT()
#endif

namespace Espfc {

#ifdef ESPFC_DEBUG_SERIAL
extern Stream* _debugStream;

static inline void initDebugStream(Stream* p)
{
  _debugStream = p;
}

#define LOG_SERIAL_INIT(p) _debugStream = p;
#define LOG_SERIAL_DEBUG(v)                                                                                            \
  if (_debugStream)                                                                                                    \
  {                                                                                                                    \
    _debugStream->print(v);                                                                                            \
  }
#define LOG_SERIAL_DEBUG_HEX(v)                                                                                        \
  if (_debugStream)                                                                                                    \
  {                                                                                                                    \
    _debugStream->print(v, HEX);                                                                                       \
  }

template<typename T>
void D(T t)
{
  if (!_debugStream) return;
  _debugStream->println(t);
}

template<typename T, typename... Args>
void D(T t, Args... args) // recursive variadic function
{
  if (!_debugStream) return;
  _debugStream->print(t);
  _debugStream->print(' ');
  D(args...);
}

#else

static inline void initDebugStream(Stream* p) {}

#define LOG_SERIAL_INIT(p)
#define LOG_SERIAL_DEBUG(v)
#define LOG_SERIAL_DEBUG_HEX(v)
#define D(...)

#endif

} // namespace Espfc

#endif
