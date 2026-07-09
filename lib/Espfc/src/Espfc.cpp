#include "Espfc.h"
#include "Hal/Gpio.h"
#include "Device/DroneProtoServo.hpp"
#include "DroneProtoProfiles.hpp"
#include "Debug_Espfc.h"

namespace Espfc {

Espfc::Espfc():
  _hardware{_model}, _controller{_model}, _telemetry{_model}, _input{_model, _telemetry}, _actuator{_model}, _sensor{_model},
  _mixer{_model}, _blackbox{_model}, _buzzer{_model}, _serial{_model, _telemetry}
  {}

int Espfc::load()
{
  PIN_DEBUG_INIT();
  _model.load();
  _model.state.appQueue.begin();
  return 1;
}

int Espfc::begin()
{
  DRONE_PROTO_DEBUG_LINE("espfc.begin start");
  _model.state.led.begin(_model.config.pin[PIN_LED_BLINK], _model.config.led.type, _model.config.led.invert);

  DRONE_PROTO_DEBUG_LINE("before serial.begin");
  _serial.begin();      // requires _model.load()
  DRONE_PROTO_DEBUG_LINE("after serial.begin, before hardware.begin");
  //_model.logStorageResult();
  _hardware.begin();    // requires _model.load()
  DRONE_PROTO_DEBUG_LINE("after hardware.begin, before model.begin");
  _model.begin();       // requires _hardware.begin()
  DRONE_PROTO_DEBUG_LINE("after model.begin, before mixer.begin");
  _mixer.begin();
  #if defined(ESP32) && defined(ESPFC_DRONE_PROTO_SERVO_PIN)
  Device::DroneProtoServo::centerDefault();
  #endif
  DRONE_PROTO_DEBUG_LINE("after mixer.begin, before sensor.begin");
  _sensor.begin();      // requires _hardware.begin()
  DRONE_PROTO_DEBUG_LINE("after sensor.begin, before input.begin");
  _input.begin();       // requires _serial.begin()
  DRONE_PROTO_DEBUG_LINE("after input.begin, before actuator.begin");
  _actuator.begin();    // requires _model.begin()
  DRONE_PROTO_DEBUG_LINE("after actuator.begin, before controller.begin");
  _controller.begin();
  DRONE_PROTO_DEBUG_LINE("after controller.begin, before blackbox.begin");
  _blackbox.begin();    // requires _serial.begin(), _actuator.begin()
  DRONE_PROTO_DEBUG_LINE("after blackbox.begin, before buzzer.begin");
  _buzzer.begin();
  DRONE_PROTO_DEBUG_LINE("after buzzer.begin");

  _model.state.buzzer.push(BUZZER_SYSTEM_INIT);

  DRONE_PROTO_DEBUG_LINE("espfc.begin done");
  return 1;
}

int FAST_CODE_ATTR Espfc::update(bool externalTrigger)
{
  if(externalTrigger)
  {
    _model.state.gyro.timer.update();
  }
  else
  {
    if(!_model.state.gyro.timer.check()) return 0;
  }
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_CPU_0);

#if defined(ESPFC_MULTI_CORE)

  _sensor.read();
  if(_model.state.input.timer.syncTo(_model.state.gyro.timer, 1u))
  {
    _input.update();
  }
  if(_model.state.actuatorTimer.check())
  {
    _actuator.update();
  }

#else

  _sensor.update();
  if(_model.state.loopTimer.syncTo(_model.state.gyro.timer))
  {
    _controller.update();
    if(_model.state.mixer.timer.syncTo(_model.state.loopTimer))
    {
      _mixer.update();
    }
    _blackbox.update();
    if(_model.state.input.timer.syncTo(_model.state.gyro.timer, 1u))
    {
      _input.update();
    }
    if(_model.state.actuatorTimer.check())
    {
      _actuator.update();
    }
  }
  _sensor.updateDelayed();

#endif

  _serial.update();
  _buzzer.update();
  _model.state.led.update();
  #if defined(ESP32) && defined(ESPFC_DRONE_PROTO_SERVO_PIN)
  Device::DroneProtoServo::update();
  #endif
  _model.state.stats.update();

  return 1;
}

int Espfc::updateSerialOnly()
{
  _serial.update();
  _buzzer.update();
  _model.state.led.update();
  #if defined(ESP32) && defined(ESPFC_DRONE_PROTO_SERVO_PIN)
  Device::DroneProtoServo::update();
  #endif
  _model.state.stats.update();
  return 1;
}

void Espfc::applyDroneProtoTargetConfig()
{
#if defined(ESPFC_TARGET_DRONE_PROTO)
#ifdef ESPFC_INPUT
  _model.config.pin[PIN_INPUT_RX] = ESPFC_INPUT_PIN;
#endif
  _model.config.pin[PIN_OUTPUT_0] = ESPFC_OUTPUT_0;
  _model.config.pin[PIN_OUTPUT_1] = ESPFC_OUTPUT_1;
  _model.config.pin[PIN_OUTPUT_2] = ESPFC_OUTPUT_2;
  _model.config.pin[PIN_OUTPUT_3] = ESPFC_OUTPUT_3;
  _model.config.pin[PIN_BUTTON] = ESPFC_BUTTON_PIN;
  _model.config.pin[PIN_BUZZER] = ESPFC_BUZZER_PIN;
  _model.config.pin[PIN_LED_BLINK] = ESPFC_LED_PIN;
#ifdef ESPFC_SERIAL_0
  _model.config.pin[PIN_SERIAL_0_TX] = ESPFC_SERIAL_0_TX;
  _model.config.pin[PIN_SERIAL_0_RX] = ESPFC_SERIAL_0_RX;
#endif
#ifdef ESPFC_SERIAL_1
  _model.config.pin[PIN_SERIAL_1_TX] = ESPFC_SERIAL_1_TX;
  _model.config.pin[PIN_SERIAL_1_RX] = ESPFC_SERIAL_1_RX;
#endif
#ifdef ESPFC_SERIAL_2
  _model.config.pin[PIN_SERIAL_2_TX] = ESPFC_SERIAL_2_TX;
  _model.config.pin[PIN_SERIAL_2_RX] = ESPFC_SERIAL_2_RX;
#endif
#ifdef ESPFC_I2C_0
  _model.config.pin[PIN_I2C_0_SCL] = ESPFC_I2C_0_SCL;
  _model.config.pin[PIN_I2C_0_SDA] = ESPFC_I2C_0_SDA;
#endif
#ifdef ESPFC_SPI_0
  _model.config.pin[PIN_SPI_0_SCK] = ESPFC_SPI_0_SCK;
  _model.config.pin[PIN_SPI_0_MOSI] = ESPFC_SPI_0_MOSI;
  _model.config.pin[PIN_SPI_0_MISO] = ESPFC_SPI_0_MISO;
  _model.config.pin[PIN_SPI_CS0] = ESPFC_SPI_CS_GYRO;
  _model.config.pin[PIN_SPI_CS1] = ESPFC_SPI_CS_BARO;
  _model.config.pin[PIN_SPI_CS2] = -1;
#endif

  _model.config.gyro.bus = BUS_SPI;
  _model.config.gyro.dev = GYRO_AUTO;
  _model.config.gyro.dlpf = GYRO_DLPF_256;
  _model.config.gyro.align = ALIGN_DEFAULT;
  _model.config.gyro.filter = FilterConfig(FILTER_PT1, 100);
  _model.config.gyro.filter2 = FilterConfig(FILTER_PT1, 100);
  _model.config.gyro.filter3 = FilterConfig(FILTER_NONE, 0);
  _model.config.gyro.notch1Filter = FilterConfig(FILTER_NOTCH, 0, 0);
  _model.config.gyro.notch2Filter = FilterConfig(FILTER_NOTCH, 0, 0);
  _model.config.gyro.dynLpfFilter = FilterConfig(FILTER_PT1, 0, 0);
  _model.config.gyro.dynamicFilter.count = 0;
#if !defined(ESPFC_DRONE_PROTO_ENABLE_DSHOT_BIDIR)
  _model.config.gyro.rpmFilter.harmonics = 0;
#endif

  _model.config.accel.bus = BUS_SPI;
#if defined(ESPFC_DRONE_PROTO_GYRO_NO_ACCEL)
  _model.config.accel.dev = GYRO_NONE;
#else
  _model.config.accel.dev = GYRO_AUTO;
#endif
  _model.config.accel.filter = FilterConfig(FILTER_PT1, 30);
#if defined(ESPFC_DRONE_PROTO_ENABLE_BMP388)
  _model.config.baro.bus = BUS_I2C;
  _model.config.baro.dev = BARO_BMP388;
#else
  _model.config.baro.dev = BARO_NONE;
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_BMM150)
  _model.config.mag.bus = BUS_I2C;
  _model.config.mag.dev = MAG_BMM150;
  _model.config.mag.align = ALIGN_DEFAULT;
#else
  _model.config.mag.dev = MAG_NONE;
#endif
#if defined(ESPFC_DRONE_PROTO_GYRO_NO_ACCEL)
  _model.config.fusion.mode = FUSION_NONE;
#else
  _model.config.fusion.mode = FUSION_COMPLEMENTARY;
#endif
  _model.config.featureMask = FEATURE_RX_SERIAL;
  _model.config.input.serialRxProvider = SERIALRX_CRSF;
  _model.config.loopSync = 1;
  _model.config.mixerSync = 1;

  for (int i = 0; i < SERIAL_UART_COUNT; i++)
  {
    _model.config.serial[i].functionMask = SERIAL_FUNCTION_NONE;
    _model.config.serial[i].baud = SERIAL_SPEED_115200;
    _model.config.serial[i].blackboxBaud = SERIAL_SPEED_NONE;
  }
#ifdef ESPFC_SERIAL_USB
  _model.config.serial[SERIAL_USB].id = SERIAL_ID_USB_VCP;
  _model.config.serial[SERIAL_USB].functionMask = SERIAL_FUNCTION_MSP;
#endif
#ifdef ESPFC_SERIAL_0
  _model.config.serial[SERIAL_UART_0].id = SERIAL_ID_UART_1;
  _model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
  _model.config.serial[SERIAL_UART_0].baud = SERIAL_SPEED_115200;
#endif
#ifdef ESPFC_SERIAL_2
  _model.config.serial[SERIAL_UART_2].id = SERIAL_ID_UART_3;
  _model.config.serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_RX_SERIAL;
  _model.config.serial[SERIAL_UART_2].baud = SERIAL_SPEED_400000;
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_DSHOT_BIDIR)
  const DroneProtoProfiles::Profile profile = DroneProtoProfiles::parse(_model.config.modelName);
  const bool flightProfile = profile == DroneProtoProfiles::PROFILE_HOVER_SAFE || profile == DroneProtoProfiles::PROFILE_ACRO_TEST;
  const bool telemetryProtocol = _model.config.output.protocol == ESC_PROTOCOL_DSHOT300 || _model.config.output.protocol == ESC_PROTOCOL_DSHOT600;
  if(flightProfile && telemetryProtocol)
  {
    _model.config.output.dshotTelemetry = true;
  }
  if(!telemetryProtocol)
  {
    _model.config.output.dshotTelemetry = false;
  }
#endif
#endif
}

void Espfc::applyDroneProtoStartupConfig()
{
#if defined(ESPFC_TARGET_DRONE_PROTO)
  if (_model.configLoadedFromStorage())
  {
    applyDroneProtoTargetConfig();
    _model.logger.info().logln(F("DRONE PROTO EEPROM CONFIG"));
    return;
  }

  forceDroneProtoBenchConfig();
  _model.logger.info().log(F("DRONE PROTO DEFAULT PROFILE ")).logln(DroneProtoProfiles::name(DroneProtoProfiles::PROFILE_HOVER_SAFE));
#endif
}

void Espfc::forceDroneProtoBenchConfig()
{
#if defined(ESPFC_TARGET_DRONE_PROTO)
#ifdef ESPFC_INPUT
  _model.config.pin[PIN_INPUT_RX] = ESPFC_INPUT_PIN;
#endif
  _model.config.pin[PIN_OUTPUT_0] = ESPFC_OUTPUT_0;
  _model.config.pin[PIN_OUTPUT_1] = ESPFC_OUTPUT_1;
  _model.config.pin[PIN_OUTPUT_2] = ESPFC_OUTPUT_2;
  _model.config.pin[PIN_OUTPUT_3] = ESPFC_OUTPUT_3;
  _model.config.pin[PIN_BUTTON] = ESPFC_BUTTON_PIN;
  _model.config.pin[PIN_BUZZER] = ESPFC_BUZZER_PIN;
  _model.config.pin[PIN_LED_BLINK] = ESPFC_LED_PIN;
#ifdef ESPFC_SERIAL_0
  _model.config.pin[PIN_SERIAL_0_TX] = ESPFC_SERIAL_0_TX;
  _model.config.pin[PIN_SERIAL_0_RX] = ESPFC_SERIAL_0_RX;
#endif
#ifdef ESPFC_SERIAL_1
  _model.config.pin[PIN_SERIAL_1_TX] = ESPFC_SERIAL_1_TX;
  _model.config.pin[PIN_SERIAL_1_RX] = ESPFC_SERIAL_1_RX;
#endif
#ifdef ESPFC_SERIAL_2
  _model.config.pin[PIN_SERIAL_2_TX] = ESPFC_SERIAL_2_TX;
  _model.config.pin[PIN_SERIAL_2_RX] = ESPFC_SERIAL_2_RX;
#endif
#ifdef ESPFC_I2C_0
  _model.config.pin[PIN_I2C_0_SCL] = ESPFC_I2C_0_SCL;
  _model.config.pin[PIN_I2C_0_SDA] = ESPFC_I2C_0_SDA;
#endif
#ifdef ESPFC_SPI_0
  _model.config.pin[PIN_SPI_0_SCK] = ESPFC_SPI_0_SCK;
  _model.config.pin[PIN_SPI_0_MOSI] = ESPFC_SPI_0_MOSI;
  _model.config.pin[PIN_SPI_0_MISO] = ESPFC_SPI_0_MISO;
  _model.config.pin[PIN_SPI_CS0] = ESPFC_SPI_CS_GYRO;
  _model.config.pin[PIN_SPI_CS1] = ESPFC_SPI_CS_BARO;
  _model.config.pin[PIN_SPI_CS2] = -1;
#endif

#if defined(ESPFC_DRONE_PROTO_BETAFLIGHT_SONAR_ONLY)
  _model.config.gyro.bus = BUS_NONE;
  _model.config.gyro.dev = GYRO_NONE;
  _model.config.accel.bus = BUS_NONE;
  _model.config.accel.dev = GYRO_NONE;
  _model.config.baro.bus = BUS_NONE;
  _model.config.baro.dev = BARO_NONE;
  _model.config.mag.bus = BUS_NONE;
  _model.config.mag.dev = MAG_NONE;
  _model.config.fusion.mode = FUSION_NONE;
  _model.config.featureMask = 0;
  _model.config.loopSync = 1;
  _model.config.mixerSync = 1;
  _model.config.output.protocol = ESC_PROTOCOL_DISABLED;
  _model.config.output.dshotTelemetry = false;
  _model.config.blackbox.dev = BLACKBOX_DEV_NONE;
  _model.config.blackbox.pDenom = 0;
  _model.config.debug.mode = DEBUG_RANGEFINDER;

  for (int i = 0; i < SERIAL_UART_COUNT; i++)
  {
    _model.config.serial[i].functionMask = SERIAL_FUNCTION_NONE;
    _model.config.serial[i].baud = SERIAL_SPEED_115200;
    _model.config.serial[i].blackboxBaud = SERIAL_SPEED_NONE;
  }
#ifdef ESPFC_SERIAL_USB
  _model.config.serial[SERIAL_USB].id = SERIAL_ID_USB_VCP;
  _model.config.serial[SERIAL_USB].functionMask = SERIAL_FUNCTION_MSP;
#endif
#ifdef ESPFC_SERIAL_0
  _model.config.serial[SERIAL_UART_0].id = SERIAL_ID_UART_1;
  _model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
  _model.config.serial[SERIAL_UART_0].baud = SERIAL_SPEED_115200;
#endif
  return;
#endif

  _model.config.gyro.bus = BUS_SPI;
  _model.config.gyro.dev = GYRO_AUTO;
  _model.config.gyro.dlpf = GYRO_DLPF_256;
  _model.config.gyro.align = ALIGN_DEFAULT;
  _model.config.gyro.filter = FilterConfig(FILTER_PT1, 100);
  _model.config.gyro.filter2 = FilterConfig(FILTER_PT1, 100);
  _model.config.gyro.filter3 = FilterConfig(FILTER_NONE, 0);
  _model.config.gyro.notch1Filter = FilterConfig(FILTER_NOTCH, 0, 0);
  _model.config.gyro.notch2Filter = FilterConfig(FILTER_NOTCH, 0, 0);
  _model.config.gyro.dynLpfFilter = FilterConfig(FILTER_PT1, 0, 0);
  _model.config.gyro.dynamicFilter.count = 0;
  _model.config.gyro.rpmFilter.harmonics = 0;

  _model.config.accel.bus = BUS_SPI;
#if defined(ESPFC_DRONE_PROTO_GYRO_NO_ACCEL)
  _model.config.accel.dev = GYRO_NONE;
#else
  _model.config.accel.dev = GYRO_AUTO;
#endif
  for (int i = 0; i < 3; i++)
  {
    _model.config.accel.bias[i] = 0;
  }
  _model.config.accel.trim[0] = 0;
  _model.config.accel.trim[1] = 0;
  _model.state.accel.bias = VectorFloat();
  _model.state.accel.calibrationState = CALIBRATION_IDLE;
  _model.config.accel.filter = FilterConfig(FILTER_PT1, 30);
#if defined(ESPFC_DRONE_PROTO_ENABLE_BMP388)
  _model.config.baro.bus = BUS_I2C;
  _model.config.baro.dev = BARO_BMP388;
#else
  _model.config.baro.dev = BARO_NONE;
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_BMM150)
  _model.config.mag.bus = BUS_I2C;
  _model.config.mag.dev = MAG_BMM150;
  _model.config.mag.align = ALIGN_DEFAULT;
#else
  _model.config.mag.dev = MAG_NONE;
#endif
#if defined(ESPFC_DRONE_PROTO_GYRO_NO_ACCEL)
  _model.config.fusion.mode = FUSION_NONE;
#else
  _model.config.fusion.mode = FUSION_COMPLEMENTARY;
#endif
  _model.config.featureMask = FEATURE_RX_SERIAL;
  _model.config.input.serialRxProvider = SERIALRX_CRSF;
  DroneProtoProfiles::apply(_model.config, DroneProtoProfiles::PROFILE_HOVER_SAFE);
  _model.config.loopSync = 1;
  _model.config.mixerSync = 1;
#if defined(ESPFC_DRONE_PROTO_ENABLE_MOTOR_TEST_DSHOT300)
  _model.config.output.protocol = ESC_PROTOCOL_DSHOT300;
  _model.config.output.async = false;
  _model.config.output.rate = 500;
  _model.config.output.minCommand = 1000;
  _model.config.output.minThrottle = 1070;
  _model.config.output.maxThrottle = 2000;
  _model.config.output.dshotIdle = 550;
#elif defined(ESPFC_DRONE_PROTO_ENABLE_MOTOR_TEST_DSHOT150)
  _model.config.output.protocol = ESC_PROTOCOL_DSHOT150;
  _model.config.output.async = false;
  _model.config.output.rate = 500;
  _model.config.output.minCommand = 1000;
  _model.config.output.minThrottle = 1070;
  _model.config.output.maxThrottle = 2000;
  _model.config.output.dshotIdle = 800;
#elif defined(ESPFC_DRONE_PROTO_ENABLE_MOTOR_TEST_PWM)
  _model.config.output.protocol = ESC_PROTOCOL_PWM;
  _model.config.output.async = true;
  _model.config.output.rate = 50;
  _model.config.output.minCommand = 1000;
  _model.config.output.minThrottle = 1070;
  _model.config.output.maxThrottle = 2000;
#else
  _model.config.output.protocol = ESC_PROTOCOL_DISABLED;
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_DSHOT_BIDIR)
  _model.config.output.dshotTelemetry = true;
#else
  _model.config.output.dshotTelemetry = false;
#endif
  _model.config.blackbox.dev = BLACKBOX_DEV_NONE;
  _model.config.blackbox.pDenom = 0;
#if defined(ESPFC_DRONE_PROTO_ENABLE_PMW3901) || defined(ESPFC_DRONE_PROTO_ENABLE_TCS34725) || defined(ESPFC_DRONE_PROTO_ENABLE_VL53L1X)
  // API 1.48 Configurator displays this numeric slot as OPTICALFLOW.
  _model.config.debug.mode = DEBUG_RANGEFINDER_QUALITY;
#elif defined(ESPFC_DRONE_PROTO_ENABLE_DSHOT_BIDIR)
  _model.config.debug.mode = DEBUG_DSHOT_RPM_TELEMETRY;
#else
  _model.config.debug.mode = DEBUG_GYRO_SCALED;
#endif

  for (int i = 0; i < SERIAL_UART_COUNT; i++)
  {
    _model.config.serial[i].functionMask = SERIAL_FUNCTION_NONE;
    _model.config.serial[i].baud = SERIAL_SPEED_115200;
    _model.config.serial[i].blackboxBaud = SERIAL_SPEED_NONE;
  }
#ifdef ESPFC_SERIAL_USB
  _model.config.serial[SERIAL_USB].id = SERIAL_ID_USB_VCP;
  _model.config.serial[SERIAL_USB].functionMask = SERIAL_FUNCTION_MSP;
#endif
#ifdef ESPFC_SERIAL_0
  _model.config.serial[SERIAL_UART_0].id = SERIAL_ID_UART_1;
  _model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
  _model.config.serial[SERIAL_UART_0].baud = SERIAL_SPEED_115200;
#endif
#ifdef ESPFC_SERIAL_2
  _model.config.serial[SERIAL_UART_2].id = SERIAL_ID_UART_3;
  _model.config.serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_RX_SERIAL;
  _model.config.serial[SERIAL_UART_2].baud = SERIAL_SPEED_400000;
#endif
#endif
}

// other task
int FAST_CODE_ATTR Espfc::updateOther()
{
#if defined(ESPFC_MULTI_CORE)
  if(_model.state.appQueue.isEmpty())
  {
    return 0;
  }
  Event e = _model.state.appQueue.receive();

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_CPU_1);

  switch(e.type)
  {
    case EVENT_GYRO_READ:
      _sensor.preLoop();
      _controller.update();
      // skip mixer and bb if earlier than half cycle, possible delay in previous iteration, 
      // to keep space to receive dshot erpm frame, but process rest
      if(_loop_next < micros())
      {
        _loop_next = micros() + _model.state.loopTimer.interval / 2;
        _mixer.update();
        _blackbox.update();
      }
      _sensor.postLoop();
      break;
    case EVENT_ACCEL_READ:
      _sensor.fusion();
      break;
    default:
      break;
      // nothing
  }
#endif

  return 1;
}

}

