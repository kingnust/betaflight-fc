#include <Arduino.h>
#include <SPI.h>
#include <Espfc.h>

#ifdef ESP32
void IRAM_ATTR serialEventRun(void) {}
#endif

namespace {

Espfc::Espfc espfc;

#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
uint32_t lastActiveDebugMs = 0;
uint32_t activeDebugLoopCount = 0;

void activeDebug(const char *message)
{
  Serial.print("ACTIVE BMI088 DEBUG: ");
  Serial.println(message);
  Serial.flush();
}
#endif

#if defined(ESPFC_DRONE_PROTO_CLEAN_BMI088_PROBE) || defined(ESPFC_DRONE_PROTO_CLEAN_BMI088_INIT_PROBE)
static constexpr uint8_t BMI088_ACCEL_CS = 7;
static constexpr uint8_t BMI088_GYRO_CS = 8;
static constexpr int8_t BMI088_SPI_SCK = 9;
static constexpr int8_t BMI088_SPI_MISO = 10;
static constexpr int8_t BMI088_SPI_MOSI = 11;
static constexpr uint32_t PROBE_START_DELAY_MS = 8000;
static constexpr uint32_t PROBE_INTERVAL_MS = 1000;

bool probeStarted = false;
uint32_t lastProbeMs = 0;

void publishProbeDebug(int16_t stage, int16_t accelId = -1, int16_t gyroId = -1)
{
  espfc.setDebugValue(0, stage);
  espfc.setDebugValue(1, accelId);
  espfc.setDebugValue(2, gyroId);
  espfc.setDebugValue(3, static_cast<int16_t>(millis() / 1000));
}

uint8_t bmi088ReadReg(uint8_t cs, uint8_t reg, bool accel)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  if (accel)
  {
    SPI.transfer(0x00);
  }
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return value;
}

void bmi088WriteReg(uint8_t cs, uint8_t reg, uint8_t value)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

#if defined(ESPFC_DRONE_PROTO_CLEAN_BMI088_INIT_PROBE)
void printIdLine(int16_t stage, const char *label)
{
  const uint8_t accelId = bmi088ReadReg(BMI088_ACCEL_CS, 0x00, true);
  const uint8_t gyroId = bmi088ReadReg(BMI088_GYRO_CS, 0x00, false);
  publishProbeDebug(stage, accelId, gyroId);
  Serial.print("CLEAN BMI088 INIT: ");
  Serial.print(label);
  Serial.print(" accel_id=0x");
  Serial.print(accelId, HEX);
  Serial.print(" gyro_id=0x");
  Serial.println(gyroId, HEX);
  Serial.flush();
}

void initBmi088StepByStep()
{
  printIdLine(10, "before init");

  publishProbeDebug(20);
  Serial.println("CLEAN BMI088 INIT: accel soft reset");
  Serial.flush();
  bmi088WriteReg(BMI088_ACCEL_CS, 0x7E, 0xB6);
  delay(60);
  printIdLine(21, "after accel reset");

  publishProbeDebug(30);
  Serial.println("CLEAN BMI088 INIT: gyro soft reset");
  Serial.flush();
  bmi088WriteReg(BMI088_GYRO_CS, 0x14, 0xB6);
  delay(60);
  printIdLine(31, "after gyro reset");

  publishProbeDebug(40);
  Serial.println("CLEAN BMI088 INIT: accel power enable");
  Serial.flush();
  bmi088WriteReg(BMI088_ACCEL_CS, 0x7D, 0x04);
  delay(10);
  bmi088WriteReg(BMI088_ACCEL_CS, 0x7C, 0x00);
  delay(10);

  publishProbeDebug(50);
  Serial.println("CLEAN BMI088 INIT: accel range/odr");
  Serial.flush();
  bmi088WriteReg(BMI088_ACCEL_CS, 0x41, 0x02);
  bmi088WriteReg(BMI088_ACCEL_CS, 0x40, 0xAC);

  publishProbeDebug(60);
  Serial.println("CLEAN BMI088 INIT: gyro range/odr/int");
  Serial.flush();
  bmi088WriteReg(BMI088_GYRO_CS, 0x0F, 0x00);
  bmi088WriteReg(BMI088_GYRO_CS, 0x10, 0x81);
  bmi088WriteReg(BMI088_GYRO_CS, 0x15, 0x80);
  delay(20);

  printIdLine(70, "after init");
}
#endif

void startProbe()
{
  publishProbeDebug(2);
  Serial.println();
  Serial.println("CLEAN BMI088 PROBE: starting SPI");
  Serial.flush();

  pinMode(BMI088_ACCEL_CS, OUTPUT);
  pinMode(BMI088_GYRO_CS, OUTPUT);
  digitalWrite(BMI088_ACCEL_CS, HIGH);
  digitalWrite(BMI088_GYRO_CS, HIGH);

  SPI.begin(BMI088_SPI_SCK, BMI088_SPI_MISO, BMI088_SPI_MOSI);

  probeStarted = true;
  lastProbeMs = 0;
  publishProbeDebug(3);
  Serial.println("CLEAN BMI088 PROBE: SPI started");
  Serial.flush();

#if defined(ESPFC_DRONE_PROTO_CLEAN_BMI088_INIT_PROBE)
  initBmi088StepByStep();
#endif
}

void updateProbe()
{
  const uint32_t now = millis();

  if (!probeStarted)
  {
    if (now < PROBE_START_DELAY_MS)
    {
      publishProbeDebug(1);
      return;
    }
    startProbe();
  }

  if (now - lastProbeMs < PROBE_INTERVAL_MS)
  {
    return;
  }
  lastProbeMs = now;

  const uint8_t accelId = bmi088ReadReg(BMI088_ACCEL_CS, 0x00, true);
  const uint8_t gyroId = bmi088ReadReg(BMI088_GYRO_CS, 0x00, false);
  publishProbeDebug(90, accelId, gyroId);

  Serial.print("CLEAN BMI088 PROBE: accel_id=0x");
  Serial.print(accelId, HEX);
  Serial.print(" gyro_id=0x");
  Serial.println(gyroId, HEX);
  Serial.flush();
}
#else
void updateProbe() {}
#endif

} // namespace

void setup()
{
#if defined(ESPFC_DRONE_PROTO_WATCHDOG_SAFE) && defined(ESP32)
  disableCore0WDT();
#endif
#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
  Serial.begin(115200);
  delay(1000);
  activeDebug("setup start");
  activeDebug("before espfc.load");
#endif
  espfc.load();
#if defined(ESPFC_DRONE_PROTO_FORCE_BENCH_CONFIG)
  espfc.forceDroneProtoBenchConfig();
#endif
#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
  activeDebug("after espfc.load, before espfc.begin");
#endif
  espfc.begin();
#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
  activeDebug("after espfc.begin");
  Serial.print("ACTIVE BMI088 DEBUG: gyro_interval_us=");
  Serial.println(espfc.getGyroInterval());
  Serial.flush();
#endif
#if defined(ESPFC_DRONE_PROTO_CLEAN_BMI088_PROBE) || defined(ESPFC_DRONE_PROTO_CLEAN_BMI088_INIT_PROBE)
  espfc.setDebugMode(Espfc::DEBUG_GYRO_SCALED);
  publishProbeDebug(1);
  Serial.println();
#if defined(ESPFC_DRONE_PROTO_CLEAN_BMI088_INIT_PROBE)
  Serial.println("CLEAN BMI088 INIT: Betaflight baseline is running; BMI088 init starts after 8s");
#else
  Serial.println("CLEAN BMI088 PROBE: Betaflight baseline is running; SPI probe starts after 8s");
#endif
  Serial.flush();
#endif
}

void loop()
{
#if defined(ESPFC_DRONE_PROTO_PASSIVE_GYRO)
  espfc.updateSerialOnly();
#else
#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
  if (activeDebugLoopCount < 20)
  {
    Serial.print("ACTIVE BMI088 DEBUG: loop ");
    Serial.print(activeDebugLoopCount);
    Serial.println(" before update");
    Serial.flush();
  }
#endif
  int updateResult = espfc.update();
#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
  if (activeDebugLoopCount < 20)
  {
    Serial.print("ACTIVE BMI088 DEBUG: loop ");
    Serial.print(activeDebugLoopCount);
    Serial.print(" after update result=");
    Serial.println(updateResult);
    Serial.flush();
  }
#endif
  espfc.updateOther();
#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
  if (activeDebugLoopCount < 20)
  {
    Serial.print("ACTIVE BMI088 DEBUG: loop ");
    Serial.print(activeDebugLoopCount);
    Serial.println(" after updateOther");
    Serial.flush();
  }
  activeDebugLoopCount++;
#endif
  updateProbe();
#endif
#if defined(ESPFC_DRONE_PROTO_ACTIVE_DEBUG)
  const uint32_t now = millis();
  if (now - lastActiveDebugMs >= 1000)
  {
    lastActiveDebugMs = now;
    Serial.print("ACTIVE BMI088 DEBUG: heartbeat_ms=");
    Serial.print(now);
    Serial.print(" gyro_interval_us=");
    Serial.println(espfc.getGyroInterval());
    Serial.flush();
  }
#endif
  delay(1);
}
