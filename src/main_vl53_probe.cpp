#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>

namespace {

VL53L1X sensor;

constexpr int SDA_PIN = 42;
constexpr int SCL_PIN = 41;
constexpr uint8_t VL53_ADDRESS = 0x29;
constexpr uint32_t I2C_SPEED = 400000;
constexpr uint16_t SENSOR_TIMEOUT_MS = 500;
constexpr uint32_t MEASUREMENT_BUDGET_US = 50000;
constexpr uint32_t CONTINUOUS_PERIOD_MS = 50;

uint32_t sampleCount = 0;

bool addressPresent(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
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

void startSensor()
{
  printI2cScan();
  if (!addressPresent(VL53_ADDRESS))
  {
    Serial.println("VL53: no device at 0x29");
    while (true) delay(1000);
  }

  sensor.setTimeout(SENSOR_TIMEOUT_MS);
  if (!sensor.init())
  {
    Serial.print("VL53: init failed timeout=");
    Serial.print(sensor.timeoutOccurred());
    Serial.print(" last_status=");
    Serial.println(sensor.last_status);
    while (true) delay(1000);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(MEASUREMENT_BUDGET_US);
  sensor.startContinuous(CONTINUOUS_PERIOD_MS);

  Serial.println("VL53 started");
  Serial.println("Columns: sample range_mm status status_text signal_mcps ambient_mcps last_i2c_status");
}

} // namespace

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("Drone Prototype sonar-only VL53L1X build");
  Serial.println("Only VL53L1X is active; no BMI/BMP/BMM/motors/Betaflight loop.");
  Serial.println("Pins: SDA=42 SCL=41 address=0x29 I2C=400kHz");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_SPEED);

  startSensor();
}

void loop()
{
  const uint16_t distance = sensor.read();

  Serial.print("sample=");
  Serial.print(sampleCount++);
  Serial.print(" range_mm=");
  Serial.print(distance);
  Serial.print(" status=");
  Serial.print((uint8_t)sensor.ranging_data.range_status);
  Serial.print(" status_text=");
  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print(" signal_mcps=");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS, 3);
  Serial.print(" ambient_mcps=");
  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS, 3);
  Serial.print(" last_i2c_status=");
  Serial.println(sensor.last_status);

  if (sensor.timeoutOccurred() || sensor.last_status != 0)
  {
    Serial.println("VL53 read fault; reboot or power-cycle before retesting.");
    while (true) delay(1000);
  }
}
