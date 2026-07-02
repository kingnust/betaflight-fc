#include <Arduino.h>

#if defined(ESPFC_DRONE_PROTO_BMI088_PROBE)

#include <SPI.h>

static constexpr uint8_t BMI088_ACCEL_CS = 7;
static constexpr uint8_t BMI088_GYRO_CS = 8;
static constexpr int8_t BMI088_SPI_SCK = 9;
static constexpr int8_t BMI088_SPI_MISO = 10;
static constexpr int8_t BMI088_SPI_MOSI = 11;

static uint8_t bmi088ReadReg(uint8_t cs, uint8_t reg, bool accel)
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

static void bmi088ReadBytes(uint8_t cs, uint8_t reg, bool accel, uint8_t *data, size_t len)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  if (accel)
  {
    SPI.transfer(0x00);
  }
  for (size_t i = 0; i < len; i++)
  {
    data[i] = SPI.transfer(0x00);
  }
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

static void printBmi088Status()
{
  const uint8_t accelId = bmi088ReadReg(BMI088_ACCEL_CS, 0x00, true);
  const uint8_t gyroId = bmi088ReadReg(BMI088_GYRO_CS, 0x00, false);
  uint8_t gyroData[6] = {};
  uint8_t accelData[6] = {};
  bmi088ReadBytes(BMI088_GYRO_CS, 0x02, false, gyroData, sizeof(gyroData));
  bmi088ReadBytes(BMI088_ACCEL_CS, 0x12, true, accelData, sizeof(accelData));

  Serial.print("BMI088 accel_id=0x");
  Serial.print(accelId, HEX);
  Serial.print(" gyro_id=0x");
  Serial.print(gyroId, HEX);
  Serial.print(" gyro_raw=");
  Serial.print((int16_t)((gyroData[1] << 8) | gyroData[0]));
  Serial.print(",");
  Serial.print((int16_t)((gyroData[3] << 8) | gyroData[2]));
  Serial.print(",");
  Serial.print((int16_t)((gyroData[5] << 8) | gyroData[4]));
  Serial.print(" accel_raw=");
  Serial.print((int16_t)((accelData[1] << 8) | accelData[0]));
  Serial.print(",");
  Serial.print((int16_t)((accelData[3] << 8) | accelData[2]));
  Serial.print(",");
  Serial.println((int16_t)((accelData[5] << 8) | accelData[4]));
}

void setup()
{
  Serial.begin(115200);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 3000)
  {
    delay(10);
  }
  Serial.println();
  Serial.println("Drone Prototype BMI088 raw SPI probe booted");
  Serial.flush();
  delay(100);

  Serial.println("Configuring BMI088 CS pins");
  pinMode(BMI088_ACCEL_CS, OUTPUT);
  pinMode(BMI088_GYRO_CS, OUTPUT);
  digitalWrite(BMI088_ACCEL_CS, HIGH);
  digitalWrite(BMI088_GYRO_CS, HIGH);
  Serial.flush();

  Serial.println("Starting SPI on SCK=9 MISO=10 MOSI=11");
  Serial.flush();
  SPI.begin(BMI088_SPI_SCK, BMI088_SPI_MISO, BMI088_SPI_MOSI);
  delay(100);

  Serial.println("Expected accel_id=0x1E gyro_id=0x0F");
  Serial.flush();
}

void loop()
{
  Serial.print("probe heartbeat ");
  Serial.println(millis());
  Serial.flush();
  printBmi088Status();
  delay(500);
}

#else

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Espfc.h>
#include <Kalman.h>
#include <Madgwick.h>
#include <Mahony.h>
#include <printf.h>
#include <blackbox/blackbox.h>
#include <EscDriver.h>
#include <EspWire.h>
#include <Gps.hpp>
#if defined(ESPFC_ESPNOW)
#include <EspNowRcLink/Receiver.h>
#endif
#ifdef ESPFC_WIFI_ALT
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif
#include "Debug_Espfc.h"

#ifdef ESP32
void IRAM_ATTR serialEventRun(void) {}
#endif

Espfc::Espfc espfc;

#if defined(ESPFC_MULTI_CORE)
  #if defined(ESPFC_FREE_RTOS)

    // ESP32 multicore
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <driver/timer.h>

    TaskHandle_t gyroTaskHandle = NULL;
    TaskHandle_t pidTaskHandle = NULL;
    static const timer_group_t TIMER_GROUP = TIMER_GROUP_0;
    static const timer_idx_t TIMER_IDX = TIMER_0;

    bool IRAM_ATTR gyroTimerIsr(void* args)
    {
      BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(gyroTaskHandle, &xHigherPriorityTaskWoken);
      return xHigherPriorityTaskWoken == pdTRUE;
    }

    void gyroTimerInit(bool (*isrCb)(void* args), int interval)
    {
      if(interval <= 0)
      {
        interval = 1000;
      }
      timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,
      };
      timer_init(TIMER_GROUP, TIMER_IDX, &config);
      timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0);
      timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, interval);
      timer_isr_callback_add(TIMER_GROUP, TIMER_IDX, isrCb, nullptr, ESP_INTR_FLAG_IRAM);
      timer_enable_intr(TIMER_GROUP, TIMER_IDX);
      timer_start(TIMER_GROUP, TIMER_IDX);
    }

    void gyroTask(void *pvParameters)
    {
      espfc.begin();
      gyroTimerInit(gyroTimerIsr, espfc.getGyroInterval());
      while(true)
      {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for timer isr notification
        espfc.update(true);
      }
    }

    void pidTask(void *pvParameters)
    {
      while(true)
      {
        espfc.updateOther();
      }
    }

    void setup()
    {
      disableCore0WDT();
      // internal task priorities
      // PRO(0): hi-res timer(22), timer(1), event-loop(20), lwip(18/any), wifi(23), wpa(2/any), BT/vhci(23), NimBle(21), BT/other(19,20,22), Eth(15), Mqtt(5/any)
      // APP(1): free
      espfc.load();
      xTaskCreateUniversal(gyroTask, "gyroTask", 8192, NULL, 24, &gyroTaskHandle, 1);
      xTaskCreateUniversal(pidTask,  "pidTask",  8192, NULL,  1, &pidTaskHandle,  0);
      vTaskDelete(NULL); // delete arduino loop task
    }

    void loop()
    {
    }

  #elif defined(ESPFC_MULTI_CORE_RP2040)

    bool core1_separate_stack = true;
    volatile bool setup_done = false;

    // RP2040 multicore
    // TODO: https://emalliab.wordpress.com/2021/04/18/raspberry-pi-pico-arduino-core-and-timers/
    void setup()
    {
      espfc.load();
      espfc.begin();
      setup_done = true;
    }
    void loop()
    {
      espfc.update();
    }
    void setup1()
    {
      while(!setup_done);
    }
    void loop1()
    {
      espfc.updateOther();
    }

  #else
    #error "No RTOS defined for multicore board"
  #endif

#else

  // single core
  void setup()
  {
    espfc.load();
    espfc.begin();
  }
  void loop()
  {
    espfc.update();
    espfc.updateOther();
#if defined(ESPFC_DRONE_PROTO_WATCHDOG_SAFE)
    delay(1);
#endif
  }

#endif

#endif
