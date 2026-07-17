#include "Device/Baro/BaroBMP085.hpp"
#include "Device/Baro/BaroBMP280.hpp"
#include "Device/Baro/BaroBMP388.hpp"
#include "Device/Baro/BaroSPL06.hpp"
#include "Device/BaroDevice.hpp"
#include "Device/GyroBMI088.h"
#include "Device/GyroBMI160.h"
#include "Device/GyroDevice.h"
#include "Device/GyroICM20602.h"
#include "Device/GyroICM42688.h"
#include "Device/GyroLSM6DSO.h"
#include "Device/GyroMPU6050.h"
#include "Device/GyroMPU6500.h"
#include "Device/GyroMPU9250.h"
#include "Device/Mag/MagAK8963.hpp"
#include "Device/Mag/MagBMM150.hpp"
#include "Device/Mag/MagHMC5883L.hpp"
#include "Device/Mag/MagQMC5883L.hpp"
#include "Device/Mag/MagQMC5883P.hpp"
#include "Hal/Gpio.h"
#include "Hardware.h"
#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif
#if defined(ESPFC_DRONE_PROTO_ENABLE_DIRECT_WIFI_RC)
#include <esp_private/system_internal.h>
#endif

namespace {
#if defined(ESPFC_SPI_0)
#if defined(ESP32C3) || defined(ESP32S3) || defined(ESP32S2)
static SPIClass SPI1(HSPI);
#elif defined(ESP32)
static SPIClass SPI1(VSPI);
#endif
static Espfc::Device::BusSPI spiBus(ESPFC_SPI_0_DEV);
#endif
#if defined(ESPFC_I2C_0)
static Espfc::Device::BusI2C i2cBus(WireInstance);
#endif
static Espfc::Device::BusSlave gyroSlaveBus;
static Espfc::Device::GyroMPU6050 mpu6050;
static Espfc::Device::GyroMPU6500 mpu6500;
static Espfc::Device::GyroMPU9250 mpu9250;
static Espfc::Device::GyroLSM6DSO lsm6dso;
static Espfc::Device::GyroICM20602 icm20602;
static Espfc::Device::GyroICM42688 icm42688;
static Espfc::Device::GyroBMI088 bmi088;
static Espfc::Device::GyroBMI160 bmi160;
static Espfc::Device::Mag::MagHMC5883L hmc5883l;
static Espfc::Device::Mag::MagQMC5883L qmc5883l;
static Espfc::Device::Mag::MagQMC5883P qmc5883p;
static Espfc::Device::Mag::MagAK8963 ak8963;
static Espfc::Device::Mag::MagBMM150 bmm150;
static Espfc::Device::Baro::BaroBMP085 bmp085;
static Espfc::Device::Baro::BaroBMP280 bmp280;
static Espfc::Device::Baro::BaroBMP388 bmp388;
static Espfc::Device::Baro::BaroSPL06 spl06;
} // namespace

namespace Espfc::Device {

#if defined(ESPFC_SPI_0)
BusSPI* getMainSpiBus()
{
  return &spiBus;
}
#endif

#if defined(ESPFC_I2C_0)
BusI2C* getMainI2cBus()
{
  return &i2cBus;
}
#endif

} // namespace Espfc::Device

namespace Espfc {

Hardware::Hardware(Model& model): _model(model) {}

int Hardware::begin()
{
  DRONE_PROTO_DEBUG_LINE("hardware.begin start");
#if defined(ESPFC_DRONE_PROTO_SAFE_BOOT)
  _model.state.gyro.present = false;
  _model.state.accel.present = false;
  _model.state.mag.present = false;
  _model.state.baro.present = false;
  _model.logger.info().logln(F("DRONE PROTO SAFE BOOT"));
  DRONE_PROTO_DEBUG_LINE("hardware safe boot return");
  return 1;
#endif
  DRONE_PROTO_DEBUG_LINE("hardware before initBus");
  initBus();
  DRONE_PROTO_DEBUG_LINE("hardware before detectGyro");
  detectGyro();
  DRONE_PROTO_DEBUG_VALUE("hardware gyro_present", _model.state.gyro.present);
#if defined(ESPFC_DRONE_PROTO_GYRO_ONLY)
  DRONE_PROTO_DEBUG_LINE("hardware gyro-only return");
  return 1;
#endif
  detectMag();
  detectBaro();
  return 1;
}

void Hardware::onI2CError()
{
  _model.state.i2cErrorCount++;
  _model.state.i2cErrorDelta++;
}

void Hardware::initBus()
{
#if defined(ESPFC_SPI_0)
  DRONE_PROTO_DEBUG_LINE("initBus SPI begin");
  int spiResult = spiBus.begin(_model.config.pin[PIN_SPI_0_SCK], _model.config.pin[PIN_SPI_0_MOSI],
                               _model.config.pin[PIN_SPI_0_MISO]);
#if defined(ESPFC_DRONE_PROTO_ENABLE_PMW3901) && defined(ESPFC_PMW3901_CS)
  Hal::Gpio::pinMode(ESPFC_PMW3901_CS, OUTPUT);
  Hal::Gpio::digitalWrite(ESPFC_PMW3901_CS, HIGH);
#endif
  _model.logger.info()
      .log(F("SPI"))
      .log(_model.config.pin[PIN_SPI_0_SCK])
      .log(_model.config.pin[PIN_SPI_0_MOSI])
      .log(_model.config.pin[PIN_SPI_0_MISO])
      .logln(spiResult);
  DRONE_PROTO_DEBUG_VALUE("initBus spiResult", spiResult);
#endif
#if defined(ESPFC_I2C_0) && !defined(ESPFC_DRONE_PROTO_SKIP_I2C)
  int i2cResult =
      i2cBus.begin(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], _model.config.i2cSpeed * 1000ul);
  i2cBus.onError = std::bind(&Hardware::onI2CError, this);
  _model.logger.info()
      .log(F("I2C"))
      .log(_model.config.pin[PIN_I2C_0_SDA])
      .log(_model.config.pin[PIN_I2C_0_SCL])
      .log(_model.config.i2cSpeed)
      .logln(i2cResult);
#endif
}

void Hardware::detectGyro()
{
  DRONE_PROTO_DEBUG_LINE("detectGyro start");
  if (_model.config.gyro.dev == GYRO_NONE) return;

  Device::GyroDevice* detectedGyro = nullptr;
#if defined(ESPFC_SPI_0)
  if (_model.config.pin[PIN_SPI_CS0] != -1)
  {
    DRONE_PROTO_DEBUG_VALUE("detectGyro accel_cs", _model.config.pin[PIN_SPI_CS0]);
    DRONE_PROTO_DEBUG_VALUE("detectGyro gyro_cs", _model.config.pin[PIN_SPI_CS1]);
    Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS0], HIGH);
    Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS0], OUTPUT);
    if (_model.config.pin[PIN_SPI_CS1] != -1)
    {
      Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS1], HIGH);
      Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS1], OUTPUT);
      bmi088.setGyroCs(_model.config.pin[PIN_SPI_CS1]);
      DRONE_PROTO_DEBUG_LINE("detectGyro before bmi088 detectDevice");
      if (!detectedGyro && detectDevice(bmi088, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &bmi088;
      DRONE_PROTO_DEBUG_VALUE("detectGyro bmi088_detected", detectedGyro == &bmi088);
    }
#if !defined(ESPFC_DRONE_PROTO_BMI088_ONLY)
    if (!detectedGyro && detectDevice(mpu9250, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &mpu9250;
    if (!detectedGyro && detectDevice(mpu6500, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &mpu6500;
    if (!detectedGyro && detectDevice(icm20602, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &icm20602;
    if (!detectedGyro && detectDevice(icm42688, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &icm42688;
    if (!detectedGyro && detectDevice(bmi160, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &bmi160;
    if (!detectedGyro && detectDevice(lsm6dso, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &lsm6dso;
#endif
    if (detectedGyro && detectedGyro->getType() != GYRO_BMI088) gyroSlaveBus.begin(&spiBus, detectedGyro->getAddress());
  }
#endif
#if !defined(ESPFC_DRONE_PROTO_BMI088_ONLY)
#if defined(ESPFC_I2C_0)
  if (!detectedGyro && _model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedGyro && detectDevice(mpu9250, i2cBus)) detectedGyro = &mpu9250;
    if (!detectedGyro && detectDevice(mpu6500, i2cBus)) detectedGyro = &mpu6500;
    if (!detectedGyro && detectDevice(icm20602, i2cBus)) detectedGyro = &icm20602;
    if (!detectedGyro && detectDevice(bmi160, i2cBus)) detectedGyro = &bmi160;
    if (!detectedGyro && detectDevice(mpu6050, i2cBus)) detectedGyro = &mpu6050;
    if (!detectedGyro && detectDevice(lsm6dso, i2cBus)) detectedGyro = &lsm6dso;
    if (detectedGyro) gyroSlaveBus.begin(&i2cBus, detectedGyro->getAddress());
  }
#endif
#endif
  if (!detectedGyro)
  {
    DRONE_PROTO_DEBUG_LINE("detectGyro none");
    return;
  }

  DRONE_PROTO_DEBUG_VALUE("detectGyro type", detectedGyro->getType());
  detectedGyro->setDLPFMode(_model.config.gyro.dlpf);
  _model.state.gyro.dev = detectedGyro;
  _model.state.gyro.present = (bool)detectedGyro;
#if defined(ESPFC_DRONE_PROTO_GYRO_NO_ACCEL)
  _model.state.accel.present = false;
#else
  _model.state.accel.present = _model.state.gyro.present && _model.config.accel.dev != GYRO_NONE;
#endif
  _model.state.gyro.clock = detectedGyro->getRate();
  DRONE_PROTO_DEBUG_VALUE("detectGyro clock", _model.state.gyro.clock);
}

void Hardware::detectMag()
{
  DRONE_PROTO_DEBUG_LINE("detectMag start");
  if (_model.config.mag.dev == MAG_NONE) return;

  Device::MagDevice* detectedMag = nullptr;
#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    DRONE_PROTO_DEBUG_VALUE("detectMag sda", _model.config.pin[PIN_I2C_0_SDA]);
    DRONE_PROTO_DEBUG_VALUE("detectMag scl", _model.config.pin[PIN_I2C_0_SCL]);
    if (_model.config.mag.dev == MAG_BMM150)
    {
      DRONE_PROTO_DEBUG_LINE("detectMag before bmm150 detectDevice");
      if (!detectedMag && detectDevice(bmm150, i2cBus)) detectedMag = &bmm150;
    }
    else
    {
      if (!detectedMag && detectDevice(ak8963, i2cBus)) detectedMag = &ak8963;
      if (!detectedMag && detectDevice(hmc5883l, i2cBus)) detectedMag = &hmc5883l;
      if (!detectedMag && detectDevice(qmc5883l, i2cBus)) detectedMag = &qmc5883l;
      if (!detectedMag && detectDevice(qmc5883p, i2cBus)) detectedMag = &qmc5883p;
      if (!detectedMag && detectDevice(bmm150, i2cBus)) detectedMag = &bmm150;
    }
  }
#endif
  if (gyroSlaveBus.getBus())
  {
    if (!detectedMag && detectDevice(ak8963, gyroSlaveBus)) detectedMag = &ak8963;
    if (!detectedMag && detectDevice(hmc5883l, gyroSlaveBus)) detectedMag = &hmc5883l;
    if (!detectedMag && detectDevice(qmc5883l, gyroSlaveBus)) detectedMag = &qmc5883l;
    if (!detectedMag && detectDevice(qmc5883p, gyroSlaveBus)) detectedMag = &qmc5883p;
  }
  _model.state.mag.dev = detectedMag;
  _model.state.mag.present = (bool)detectedMag;
  _model.state.mag.rate = detectedMag ? detectedMag->getRate() : 0;
  DRONE_PROTO_DEBUG_VALUE("detectMag type", detectedMag ? detectedMag->getType() : MAG_NONE);
  DRONE_PROTO_DEBUG_VALUE("detectMag rate", _model.state.mag.rate);
}

void Hardware::detectBaro()
{
  DRONE_PROTO_DEBUG_LINE("detectBaro start");
  if (_model.config.baro.dev == BARO_NONE) return;

  Device::BaroDevice* detectedBaro = nullptr;
#if defined(ESPFC_SPI_0) && !defined(ESPFC_TARGET_DRONE_PROTO)
  if (_model.config.pin[PIN_SPI_CS1] != -1)
  {
    Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS1], HIGH);
    Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS1], OUTPUT);
    if (!detectedBaro && detectDevice(bmp280, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &spl06;
  }
#endif
#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    DRONE_PROTO_DEBUG_VALUE("detectBaro sda", _model.config.pin[PIN_I2C_0_SDA]);
    DRONE_PROTO_DEBUG_VALUE("detectBaro scl", _model.config.pin[PIN_I2C_0_SCL]);
    if (_model.config.baro.dev == BARO_BMP388)
    {
      DRONE_PROTO_DEBUG_LINE("detectBaro before bmp388 detectDevice");
      if (!detectedBaro && detectDevice(bmp388, i2cBus)) detectedBaro = &bmp388;
    }
    else
    {
      if (!detectedBaro && detectDevice(bmp280, i2cBus)) detectedBaro = &bmp280;
      if (!detectedBaro && detectDevice(bmp388, i2cBus)) detectedBaro = &bmp388;
      if (!detectedBaro && detectDevice(bmp085, i2cBus)) detectedBaro = &bmp085;
      if (!detectedBaro && detectDevice(spl06, i2cBus)) detectedBaro = &spl06;
    }
  }
#endif
  if (gyroSlaveBus.getBus())
  {
    if (!detectedBaro && detectDevice(bmp280, gyroSlaveBus)) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, gyroSlaveBus)) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, gyroSlaveBus)) detectedBaro = &spl06;
  }

  _model.state.baro.dev = detectedBaro;
  _model.state.baro.present = (bool)detectedBaro;
  DRONE_PROTO_DEBUG_VALUE("detectBaro type", detectedBaro ? detectedBaro->getType() : BARO_NONE);
}

void Hardware::restart(const Model& model)
{
#if defined(ESPFC_DRONE_PROTO_ENABLE_DIRECT_WIFI_RC)
  // Do not wait on live Wi-Fi/RMT tasks during an intentional reboot. Reset
  // both CPUs and digital peripherals immediately, and preserve a software
  // reset reason for the next boot.
  (void)model;
  esp_reset_reason_set_hint(ESP_RST_SW);
  esp_restart_noos_dig();
#endif
  if (model.state.mixer.escMotor) model.state.mixer.escMotor->end();
  if (model.state.mixer.escServo) model.state.mixer.escServo->end();
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  WiFi.disconnect();
  WiFi.softAPdisconnect();
#endif
  delay(100);
  targetReset();
}

} // namespace Espfc
