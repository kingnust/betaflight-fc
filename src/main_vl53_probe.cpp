#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>

namespace {

TwoWire vl53Wire(1);
VL53L1X vl53;

constexpr int SDA_PIN = 42;
constexpr int SCL_PIN = 41;
constexpr uint8_t VL53_ADDRESS = 0x29;
constexpr uint16_t VL53_MODEL_ID = 0xEACC;
constexpr uint32_t I2C_SPEED = 100000;
constexpr uint8_t I2C_TIMEOUT_MS = 5;
constexpr uint8_t SENSOR_TIMEOUT_MS = 20;

bool sensorStarted = false;
uint32_t lastProbeMs = 0;
uint32_t lastReadMs = 0;
uint32_t lastPrintMs = 0;

bool readReg16(uint8_t address, uint16_t reg, uint8_t* data, uint8_t len)
{
  vl53Wire.beginTransmission(address);
  vl53Wire.write((uint8_t)(reg >> 8));
  vl53Wire.write((uint8_t)reg);
  if (vl53Wire.endTransmission(false) != 0) return false;
  if (vl53Wire.requestFrom(address, len) != len) return false;
  for (uint8_t i = 0; i < len; i++)
  {
    data[i] = vl53Wire.read();
  }
  return true;
}

bool addressPresent(uint8_t address)
{
  vl53Wire.beginTransmission(address);
  return vl53Wire.endTransmission() == 0;
}

void printI2cScan()
{
  Serial.print("I2C scan on SDA=");
  Serial.print(SDA_PIN);
  Serial.print(" SCL=");
  Serial.print(SCL_PIN);
  Serial.print(":");

  bool any = false;
  for (uint8_t address = 1; address < 127; address++)
  {
    if (addressPresent(address))
    {
      Serial.print(" 0x");
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      any = true;
    }
    delay(1);
  }

  if (!any) Serial.print(" none");
  Serial.println();
}

void probeSensor()
{
  sensorStarted = false;
  Serial.println("VL53 probe start");
  printI2cScan();

  if (!addressPresent(VL53_ADDRESS))
  {
    Serial.println("VL53: no device at 0x29");
    return;
  }

  uint8_t id[2] = {0, 0};
  if (!readReg16(VL53_ADDRESS, VL53L1X::IDENTIFICATION__MODEL_ID, id, sizeof(id)))
  {
    Serial.println("VL53: failed to read model id");
    return;
  }

  const uint16_t modelId = ((uint16_t)id[0] << 8) | id[1];
  Serial.print("VL53 model id: 0x");
  Serial.println(modelId, HEX);
  if (modelId != VL53_MODEL_ID)
  {
    Serial.println("VL53: wrong model id, expected 0xEACC");
    return;
  }

  vl53.setBus(&vl53Wire);
  vl53.setTimeout(SENSOR_TIMEOUT_MS);
  if (!vl53.init())
  {
    Serial.print("VL53 init failed timeout=");
    Serial.print(vl53.timeoutOccurred());
    Serial.print(" last_status=");
    Serial.println(vl53.last_status);
    return;
  }

  vl53.setDistanceMode(VL53L1X::Long);
  vl53.setMeasurementTimingBudget(50000);
  vl53.startContinuous(100);
  sensorStarted = true;
  Serial.println("VL53 started");
}

} // namespace

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("Drone Prototype VL53L1X probe");
  Serial.println("Pins: SDA=42 SCL=41 address=0x29");

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  vl53Wire.begin(SDA_PIN, SCL_PIN, I2C_SPEED);
  vl53Wire.setTimeOut(I2C_TIMEOUT_MS);

  probeSensor();
}

void loop()
{
  const uint32_t now = millis();

  if (!sensorStarted)
  {
    if (now - lastProbeMs >= 2000)
    {
      lastProbeMs = now;
      probeSensor();
    }
    return;
  }

  if (now - lastReadMs >= 100)
  {
    lastReadMs = now;
    if (vl53.dataReady())
    {
      const uint16_t distance = vl53.read(false);
      Serial.print("range_mm=");
      Serial.print(distance);
      Serial.print(" status=");
      Serial.print((uint8_t)vl53.ranging_data.range_status);
      Serial.print(" signal=");
      Serial.print(vl53.ranging_data.peak_signal_count_rate_MCPS, 3);
      Serial.print(" ambient=");
      Serial.print(vl53.ranging_data.ambient_count_rate_MCPS, 3);
      Serial.print(" last_status=");
      Serial.println(vl53.last_status);

      if (vl53.timeoutOccurred() || vl53.last_status != 0)
      {
        Serial.println("VL53 read fault, restarting probe");
        sensorStarted = false;
      }
    }
  }

  if (now - lastPrintMs >= 1000)
  {
    lastPrintMs = now;
    Serial.println("VL53 waiting for data");
  }
}
