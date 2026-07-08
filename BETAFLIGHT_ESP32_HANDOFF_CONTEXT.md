# Betaflight ESP32-S3 Drone Prototype Handoff Context

Use this file as the first message in a new Codex chat. The previous chat history did not restore correctly, so this is a standalone context dump for continuing the Betaflight ESP32-S3 drone prototype work.

## Main Goal

Integrate the sensors and hardware from the Arduino `Drone Prototype` PlatformIO project into the ESP32-S3 Betaflight/ESP-FC project so they are usable from Betaflight Configurator, preferably the current web configurator.

The prototype is an ESP32-S3-based flight controller. The desired end state is:

- BMI088 gyro and accel working over SPI.
- BMP388 barometer working over I2C.
- BMM150 magnetometer working over I2C.
- VL53L1X rangefinder/sonar working over a separate I2C bus without freezing the FC.
- PMW3901 optical flow working over SPI.
- TCS34725 color sensor working over I2C, with controllable LED pin.
- CRSF/ELRS RadioMaster receiver working on UART.
- Motors on GPIO12-GPIO15 using DSHOT300.
- A continuous-rotation MG90S 360 servo controlled from Betaflight/CLI using the same PWM concept proven in the simple Drone Prototype sketch.
- Firmware builds archived as timestamped `.bin` files in one folder.

## Important Workspace Paths

Primary Arduino prototype project:

```text
C:\Users\keena\OneDrive\Documents\PlatformIO\Projects\Drone Prototype
```

Betaflight/ESP-FC project:

```text
C:\Users\keena\OneDrive\Documents\PlatformIO\Projects\betaflight flight controller esp32\betaflight-fc
```

Known current Codex working directory when this context was generated:

```text
C:\Users\keena\OneDrive\Documents\PlatformIO\Projects\Drone Prototype
```

## Current Git/Dirty State

`Drone Prototype` currently has local modifications:

```text
 M platformio.ini
 M simulator/index.html
 M src/main.cpp
```

The current `Drone Prototype/src/main.cpp` is a standalone continuous-rotation servo test using GPIO2 and ESP32Servo-style timing. It is not the Betaflight firmware.

`betaflight-fc` currently has local modifications:

```text
 M lib/Espfc/src/Connect/Cli.cpp
 M lib/Espfc/src/Connect/MspProcessor.cpp
 M lib/Espfc/src/Device/InputCRSF.cpp
 M lib/Espfc/src/Device/InputCRSF.h
 M lib/Espfc/src/Espfc.cpp
 M lib/Espfc/src/ModelConfig.h
 M platformio.ini
?? firmware_builds/firmware_0x00_20260707_170619.bin
?? firmware_builds/firmware_0x00_20260708_103937.bin
?? lib/Espfc/src/Device/DroneProtoServo.cpp
?? lib/Espfc/src/Device/DroneProtoServo.hpp
```

Do not blindly revert these. Some of them are the current working integration state.

## Current Build Target

The current default env in `betaflight-fc/platformio.ini` is:

```ini
[platformio]
default_envs = drone_proto_esp32s3_clean_bmi088_gyro
```

The main useful environment is:

```text
drone_proto_esp32s3_clean_bmi088_gyro
```

Build command:

```powershell
C:\Users\keena\.platformio\penv\Scripts\platformio.exe run -e drone_proto_esp32s3_clean_bmi088_gyro
```

Current `platformio.ini` for that env includes:

```ini
build_src_filter =
  -<*>
  +<main_drone_proto_clean.cpp>

lib_deps =
  ${env.lib_deps}
  pololu/VL53L1X @ ^1.3.1

build_flags =
  ${env.build_flags}
  -DESP32S3
  -DESPFC_TARGET_DRONE_PROTO
  -DESPFC_COMPAT_API_VERSION_MINOR=48
  -DESPFC_COMPAT_FC_VERSION_MINOR=5
  -DESPFC_COMPAT_FC_VERSION_PATCH_LEVEL=5
  -DESPFC_VERSION=v4_5_5_webcompat
  -DESPFC_DRONE_PROTO_ENABLE_MOTOR_TEST_DSHOT300
  -DESPFC_DRONE_PROTO_BMI088_ONLY
  -DESPFC_DRONE_PROTO_ENABLE_BMM150
  -DESPFC_DRONE_PROTO_ENABLE_BMP388
  -DESPFC_DRONE_PROTO_ENABLE_VL53L1X
  -DESPFC_DRONE_PROTO_ENABLE_PMW3901
  -DESPFC_DRONE_PROTO_ENABLE_TCS34725
  -DESPFC_DRONE_PROTO_SERVO_PIN=2
  -DESPFC_DRONE_PROTO_SERVO_AUTO_STEP
  -DESPFC_DRONE_PROTO_GYRO_RATE_500
  -DESPFC_DRONE_PROTO_WATCHDOG_SAFE
  -DESPFC_DRONE_PROTO_FORCE_BENCH_CONFIG
  -DCONFIG_FREERTOS_UNICORE
  -DARDUINO_USB_MODE=1
  -DARDUINO_USB_CDC_ON_BOOT=1

upload_port = COM5
monitor_port = COM5
```

Current known upload/monitor port is COM5. Earlier it was sometimes COM6. This is normal on ESP32-S3/Windows because bootloader and application USB CDC can enumerate as different COM ports.

## Firmware Output

The merge script is:

```text
betaflight-fc/bin/pio_merge_firmware.py
```

It creates:

```text
.pio/build/<env>/firmware_0x00.bin
.pio/build/<env>/firmware_0x00_YYYYMMDD_HHMMSS.bin
firmware_builds/firmware_0x00_YYYYMMDD_HHMMSS.bin
```

Use the timestamped `.bin` in:

```text
betaflight-fc/firmware_builds
```

Known firmware notes from testing:

- `firmware_0x00_20260703_103549.bin` was reported by the user as working during the VL53 recovery phase.
- `firmware_0x00_20260707_111818-ish` status logs showed BMI/BMP/BMM/VL53/PMW working after PMW wiring.
- Latest file seen at context generation:

```text
firmware_builds/firmware_0x00_20260708_103937.bin
```

The user prefers one `firmware_builds` folder with timestamp-only filenames, not many folders.

## Flashing

User has used the Espressif web flasher:

```text
https://espressif.github.io/esptool-js/
```

The merged `firmware_0x00_YYYYMMDD_HHMMSS.bin` should be flashed at `0x0` because it already contains bootloader, partitions, and app.

PlatformIO upload may use COM5:

```powershell
C:\Users\keena\.platformio\penv\Scripts\platformio.exe run -e drone_proto_esp32s3_clean_bmi088_gyro -t upload
```

Serial monitor:

```powershell
C:\Users\keena\.platformio\penv\Scripts\platformio.exe device monitor -p COM5 -b 115200
```

## Hardware Pin Map

Current actual code pin map is in:

```text
betaflight-fc/lib/Espfc/src/Target/TargetESP32s3.h
```

Important current values:

```text
SPI bus:
  SCK      GPIO9
  MISO     GPIO10
  MOSI     GPIO11

BMI088:
  accel CS GPIO7
  gyro CS  GPIO8
  Note: in this ESP-FC target, ESPFC_SPI_CS_BARO is repurposed as BMI088 gyro CS.

Main I2C bus for BMP388/BMM150/TCS34725:
  SCL      GPIO16
  SDA      GPIO17
  Soft I2C enabled

VL53L1X separate I2C bus:
  SCL      GPIO41
  SDA      GPIO42
  Important: VL53L1X is not on the same I2C bus as BMP/BMM/TCS.

TCS34725 LED:
  Current code macro: GPIO18
  Older docs/example said GPIO19.
  Trust current code unless the hardware has changed.

PMW3901 optical flow:
  SPI bus SCK/MISO/MOSI same as above
  CS       GPIO40

ELRS/CRSF receiver UART2:
  FC RX    GPIO38
  FC TX    GPIO37
  Wiring: receiver TX -> FC RX GPIO38, receiver RX -> FC TX GPIO37.
  If the user says "RX of receiver goes to 37", that is correct because receiver RX should connect to FC TX.

Motors:
  M1       GPIO12
  M2       GPIO13
  M3       GPIO14
  M4       GPIO15

Servo:
  Current Betaflight build flag: GPIO2
  Earlier tried GPIO4, then GPIO5, GPIO3, then GPIO2.
  GPIO2 is the current intended pin.

Avoid:
  Strapping pins: GPIO0, GPIO3, GPIO45, GPIO46
  Flash/PSRAM reserved range as noted in code: GPIO26-GPIO37
  USB/JTAG: GPIO19, GPIO20
```

## Arduino Prototype Examples

The Arduino project has known-working examples under:

```text
Drone Prototype/examples
```

Relevant examples:

```text
main BMI088.cpp
main BMP388.cpp
main BMM150.cpp
main VL53L1X.cpp
main PMW3901.cpp
main TCS34725.cpp
main drone elrs simulation.cpp
main gyro visualize.cpp
main 6 sensors.cpp
main madgwick.cpp
main Reefwing AHRS.cpp
```

The current `Drone Prototype/platformio.ini` uses:

```ini
board = 4d_systems_esp32s3_gen4_r8n16
framework = arduino
monitor_speed = 115200
build_flags =
  -DARDUINO_USB_CDC_ON_BOOT=0
  -DBOARD_NAME="ESP32S3"
  -include include/bmi088_esp32_fix.h
lib_deps =
  adafruit/Adafruit NeoPixel
  adafruit/Adafruit BMP3XX Library
  bolderflight/Bolder Flight Systems BMI088
  bitcraze/Bitcraze PMW3901
  adafruit/Adafruit TCS34725
  dfrobot/DFRobot_BMM150
  pololu/VL53L1X
  reefwing-software/Reefwing_imuTypes
  reefwing-software/ReefwingAHRS
  alfredosystems/AlfredoCRSF
  madhephaestus/ESP32Servo
```

Important known-working VL53 example:

```cpp
#define I2C_SDA 42
#define I2C_SCL 41
Wire.begin(I2C_SDA, I2C_SCL);
Wire.setClock(400000);
sensor.setTimeout(500);
sensor.init();
sensor.setDistanceMode(VL53L1X::Long);
sensor.setMeasurementTimingBudget(50000);
sensor.startContinuous(50);
sensor.read();
```

Important known-working PMW3901 example:

```cpp
#define SPI_SCK 9
#define SPI_MISO 10
#define SPI_MOSI 11
#define PMW3901_CS 40
SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, PMW3901_CS);
Bitcraze_PMW3901 flow(PMW3901_CS);
flow.begin();
flow.readMotionCount(&deltaX, &deltaY);
```

Important known-working TCS example:

```cpp
#define I2C_SDA 17
#define I2C_SCL 16
#define TCS_LED 19
Wire.begin(I2C_SDA, I2C_SCL);
```

But again, current Betaflight target macro uses TCS LED GPIO18, not GPIO19.

Important ELRS example:

```cpp
#define CRSF_RX_PIN 38
#define CRSF_TX_PIN 37
HardwareSerial crsfSerial(1);
crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
```

## Sensor Integration Files In Betaflight Project

Important files:

```text
src/main.cpp
src/main_drone_proto_clean.cpp
src/main_vl53_probe.cpp
platformio.ini
bin/pio_merge_firmware.py
bin/pio_patch_vl53.py
docs/drone-prototype.md

lib/Espfc/src/Target/TargetESP32s3.h
lib/Espfc/src/Hardware.cpp
lib/Espfc/src/Espfc.cpp
lib/Espfc/src/Model.h
lib/Espfc/src/ModelConfig.h
lib/Espfc/src/ModelState.h

lib/Espfc/src/Device/GyroBMI088.h
lib/Espfc/src/Device/BaroDevice.cpp
lib/Espfc/src/Device/Baro/BaroBMP388.cpp
lib/Espfc/src/Device/MagDevice.cpp
lib/Espfc/src/Device/Mag/MagBMM150.cpp
lib/Espfc/src/Device/ColorTCS34725.cpp
lib/Espfc/src/Device/ColorTCS34725.hpp
lib/Espfc/src/Device/OpticalFlow/OpticalFlowPMW3901.cpp
lib/Espfc/src/Device/OpticalFlow/OpticalFlowPMW3901.hpp
lib/Espfc/src/Sensor/AuxSensor.cpp
lib/Espfc/src/Sensor/AuxSensor.hpp

lib/Espfc/src/Connect/Cli.cpp
lib/Espfc/src/Connect/MspProcessor.cpp
lib/Espfc/src/Device/InputCRSF.cpp
lib/Espfc/src/Device/InputCRSF.h

lib/Espfc/src/Device/DroneProtoServo.cpp
lib/Espfc/src/Device/DroneProtoServo.hpp
```

## What Fixed The Reboot Problem Before

The ESP32-S3 reboot issue usually looked like:

```text
rst:0x8 (TG1WDT_SYS_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x4037bdb5 / 0x4037bde1
```

The important fixes that got BMI088/Betaflight stable were:

1. Stop stacking random fixes onto the full build. Start from a clean controlled entrypoint:

```text
src/main_drone_proto_clean.cpp
```

2. Use the clean env:

```text
drone_proto_esp32s3_clean_bmi088_gyro
```

3. Use `CONFIG_FREERTOS_UNICORE` and `ESPFC_DRONE_PROTO_WATCHDOG_SAFE`.

4. Add a small yield/delay path in the update loop when watchdog-safe mode is enabled:

```cpp
espfc.update();
espfc.updateOther();
#if defined(ESPFC_DRONE_PROTO_WATCHDOG_SAFE)
delay(1);
#endif
```

5. Keep BMI088 on SPI, not I2C. The working raw probe showed:

```text
CLEAN BMI088 PROBE: accel_id=0x1E gyro_id=0x0F
```

6. Avoid blocking I2C or sensor initialization inside the real-time flight loop. The VL53L1X especially caused freezes/reboots when it blocked the main sensor path.

7. For VL53, move the rangefinder polling into an auxiliary FreeRTOS task and use a separate I2C bus.

## Current VL53L1X Implementation

The VL53L1X code is currently in:

```text
lib/Espfc/src/Sensor/AuxSensor.cpp
```

It is not a full native Betaflight rangefinder driver. It is an auxiliary sensor feeding MSP/CLI state.

Current behavior:

- Uses `ESPFC_DRONE_PROTO_ENABLE_VL53L1X`.
- `AuxSensor.hpp` maps that to `ESPFC_DRONE_PROTO_AUX_VL53L1X`.
- Uses `Wire` on `ESPFC_VL53_I2C_SDA=42`, `ESPFC_VL53_I2C_SCL=41`.
- Starts a FreeRTOS task named `vl53Task`.
- Boot delay: 1500 ms.
- I2C speed in current code: 100 kHz.
- Sensor timeout: 500 ms.
- Update interval: 500 ms.
- Measurement budget: 50000 us.
- Accepts model IDs `0xEACC` and `0xEAAA`.

The script:

```text
bin/pio_patch_vl53.py
```

patches the Pololu library inside `.pio/libdeps` so `VL53L1X::init()` accepts both `0xEACC` and `0xEAAA`.

Important possible issue in current VL53 code:

When `ESPFC_DRONE_PROTO_SONAR_DEBUG` is not defined, the current macros in `AuxSensor.cpp` are redefined to:

```cpp
#define SONAR_DEBUG_LINE(v) do { yield(); vTaskDelay(pdMS_TO_TICKS(10)); } while(0)
```

and similarly for debug values/hex. This means calls that look like disabled debug lines still yield and delay 10 ms. That may have been used to avoid watchdog resets while debugging, but it is suspicious and should be reviewed. If it causes slow polling or timing side effects, replace with explicit small yields/delays only where needed rather than hiding delay inside debug macros.

Known working status example after VL53 recovery:

```text
devices: BMI088/SPI, BMP388/I2C, BMM150/I2C, VL53L1X/RANGE
aux: range=74mm status=0(valid)
```

Known issue:

- Reading sonar/range data repeatedly could freeze or cause reboot if VL53 was blocking the main path.
- The current separate task approach was created to avoid freezing other sensors.

## Current PMW3901 Optical Flow Implementation

Files:

```text
lib/Espfc/src/Device/OpticalFlow/OpticalFlowPMW3901.cpp
lib/Espfc/src/Device/OpticalFlow/OpticalFlowPMW3901.hpp
lib/Espfc/src/Sensor/AuxSensor.cpp
```

Current behavior:

- Uses SPI bus shared with BMI088.
- CS GPIO40.
- SPI mode 3.
- Clock 4 MHz.
- Detects chip ID `0x49` and inverse ID currently checked against `0xB8` in code:

```cpp
if (_chipId != 0x49 && _inverseChipId != 0xB8) return false;
```

This condition is suspicious because it uses `&&`. It only fails when both are wrong. If one is right and one is wrong, it passes. If strict checking is desired it should likely be `||`.

User status after wiring PMW:

```text
devices: BMI088/SPI, BMP388/I2C, BMM150/I2C, VL53L1X/RANGE, PMW3901/FLOW
aux: range=47mm status=0(valid) pmw_id=0x49/0xB6 flow=0/0 frames=1040
```

The user asked to combine VL53 and PMW as the optical flow module. Current state exposes them as auxiliary readouts:

- VL53L1X: rangefinder/sonar MSP bit and `MSP_SONAR_ALTITUDE`.
- PMW3901: debug/status flow deltas.

This ESP-FC tree does not have a full Betaflight navigation stack consuming optical flow for position hold. The integration is data exposure, not autonomous navigation.

## Current TCS34725 Implementation

Files:

```text
lib/Espfc/src/Device/ColorTCS34725.cpp
lib/Espfc/src/Device/ColorTCS34725.hpp
lib/Espfc/src/Sensor/AuxSensor.cpp
lib/Espfc/src/Connect/Cli.cpp
```

Current behavior:

- I2C address `0x29`.
- Uses main I2C bus on GPIO17/GPIO16.
- Integration time equivalent to `0x00` / about 614 ms.
- Gain 1x.
- LED pin from `ESPFC_TCS_LED_PIN`, currently GPIO18 in `TargetESP32s3.h`.
- CLI command:

```text
tcsled 1
tcsled 0
```

Debug mode:

```text
set debug_mode ADC_INTERNAL
```

Then debug values are intended to be:

```text
debug[0] = red
debug[1] = green
debug[2] = blue
debug[3] = clear
```

## Current CLI/Configurator Readouts

CLI `status` was extended to show devices and raw sensor values:

```text
devices: BMI088/SPI, BMP388/I2C, BMM150/I2C, VL53L1X/RANGE, PMW3901/FLOW, TCS34725/COLOR
active: acc=1 mag=1 baro=1
sensors: acc_raw=... acc=... mag=... baro=... alt_raw=... alt=... vario=...
aux: range=...mm status=...(valid) pmw_id=0x49/0xB6 flow=... frames=... color=R/G/B/C led=...
```

MSP status sensor bitmask was changed so rangefinder appears as the sonar/range bit:

```cpp
_model.rangefinderActive() << 4
```

`MSP_SONAR_ALTITUDE` returns range in centimeters if the VL53 reading is fresh:

```cpp
const bool fresh = _model.rangefinderActive() && (millis() - _model.state.aux.range.lastUpdate) < 1000;
const int32_t sonarCm = fresh ? (distanceMm + 5) / 10 : -1;
```

Debug modes:

```text
set debug_mode RANGEFINDER
```

Intended:

```text
debug[0] = range mm
debug[1] = range status
debug[2] = PMW3901 deltaX
debug[3] = PMW3901 deltaY
```

```text
set debug_mode RANGEFINDER_QUALITY
```

Intended:

```text
debug[0] = range status
debug[1] = signal
debug[2] = ambient
```

```text
set debug_mode ADC_INTERNAL
```

Intended:

```text
debug[0] = TCS red
debug[1] = TCS green
debug[2] = TCS blue
debug[3] = TCS clear
```

The user previously asked:

```text
set debug_mode GYRO_SCALED
set debug_axis 1
set blackbox_log_debug 1
```

But in this codebase, `blackbox_log_debug` may not be a real CLI setting unless implemented elsewhere.

## Web Betaflight Configurator Compatibility

The user asked to make it work with the web Betaflight Configurator and it eventually connected.

The current env uses:

```ini
-DESPFC_COMPAT_API_VERSION_MINOR=48
-DESPFC_COMPAT_FC_VERSION_MINOR=5
-DESPFC_COMPAT_FC_VERSION_PATCH_LEVEL=5
-DESPFC_VERSION=v4_5_5_webcompat
```

`MspProcessor.cpp` has compatibility additions for newer MSP commands and board info. Do not assume this is real official Betaflight. It is ESP-FC trying to speak Betaflight-compatible MSP.

## Motors / ESC / DSHOT

Motors are physically on:

```text
GPIO12, GPIO13, GPIO14, GPIO15
```

The user said changing schematic is possible, but current code uses these pins.

Current goal for motors:

- DSHOT300.
- Motors can spin fast now.
- Motors are stable.
- Direction is correct.
- Receiver can come later.
- User wanted to test bidirectional DSHOT after DSHOT300 became stable.

Important history:

- ESCs are HAKRC 25A BLS.
- ESC Configurator showed `G-H-20 - BLHeli_S, 16.7`.
- User flashed Bluejay.
- Bluejay version shown: `0.21.0`.
- Screenshot showed `G-H-20 - Bluejay, 0.21.0, 48kHz`.
- Changing to 24 kHz did not solve the earlier vibration/slow spin.
- Changing Betaflight side back to DSHOT300 fixed motor spinning.
- Bidirectional DSHOT was discussed but not confirmed as implemented/stable.

Be careful:

- Bidirectional DSHOT requires ESC firmware support and correct FC implementation for telemetry.
- The code has `#define ESPFC_DSHOT_TELEMETRY` in the target, but that does not guarantee bidirectional DSHOT is fully working.
- If enabling bidirectional, do it as a separate build flag such as `ESPFC_DRONE_PROTO_ENABLE_DSHOT_BIDIR`, test RPM telemetry in Configurator, and keep a non-bidir DSHOT300 build available.

## Receiver / CRSF / ELRS

Receiver pins:

```text
FC RX GPIO38
FC TX GPIO37
```

Correct wiring:

```text
receiver TX -> FC RX GPIO38
receiver RX -> FC TX GPIO37
GND shared
receiver power supplied
```

User reported:

- RadioMaster receiver solid blue.
- RadioMaster Pocket acknowledged/bound.
- It did not show data in monitor at one point.
- User wants receiver channels CH1 to CH32 if possible, otherwise CH1 to CH26.

Current pending code changes:

`InputCRSF.h`:

```cpp
static constexpr size_t CHANNELS = 32;
```

`InputCRSF.cpp` was modified to parse:

```text
CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED
```

and decode subset channels using the starting channel and resolution bits.

`ModelConfig.h` was extended from AUX1-AUX12 to AUX1-AUX28:

```cpp
AXIS_AUX_13 ... AXIS_AUX_28
```

Important caution:

- This 32-channel CRSF work is probably incomplete.
- Need to verify `INPUT_CHANNELS`, arrays in `ModelState`, channel mapping, MSP mode ranges, CLI input printing, and Configurator support.
- Betaflight UI may not expose 32 channels even if CRSF frames have them.
- The user requested up to CH32, but safe first step is verify CH1-CH16 still works, then CH17-CH32 by CLI/status/debug or custom print.

## Servo

The user has a continuous-rotation MG90S 360 servo, not a positional servo.

Important behavior:

- `1500 us` is stop/neutral.
- Above 1500 us rotates one direction.
- Below 1500 us rotates the other direction.
- It does not hold exact angles. It only changes speed/direction.
- To approximate 30-degree movement, pulse it for a calibrated duration, then stop.

Current standalone Arduino `Drone Prototype/src/main.cpp` does:

```cpp
SERVO_PIN = 2
SERVO_STOP_US = 1500
SERVO_RUN_US = 1600
FULL_ROTATION_MS = 2400
STEP_DEGREES = 30
STEP_INTERVAL_MS = 1000
STEP_RUN_MS = FULL_ROTATION_MS * STEP_DEGREES / 360
```

This works conceptually but is not precise. It depends on voltage, load, servo calibration, and actual speed.

Current Betaflight servo implementation:

```text
lib/Espfc/src/Device/DroneProtoServo.cpp
lib/Espfc/src/Device/DroneProtoServo.hpp
```

It uses ESP32 LEDC directly:

```cpp
LEDC_CHANNEL = 15
FREQ_HZ = 50
RES_BITS = 16
STOP = 1500 us
RUN = 1600 us
FULL_ROTATION_MS = 2400
STEP_DEGREES = 30
STEP_INTERVAL_MS = 1000
```

CLI command added:

```text
servo
servo 1500
servo 1600
servo <pin> <us>
servo step
servo stop
servo off
```

`Espfc.cpp` currently auto-starts step mode if:

```text
ESPFC_DRONE_PROTO_SERVO_AUTO_STEP
```

is defined. It is currently defined in `platformio.ini`.

MSP servo support was added in `MspProcessor.cpp`:

- `MSP_SERVO`
- `MSP_SERVO_CONFIGURATIONS`
- `MSP_SET_SERVO_CONFIGURATION`

This is meant to make the Betaflight servo tab show and configure servo 0, but this is not guaranteed to be complete. Betaflight servo tab support depends on mixer/model expectations too.

The user asked: "make the Betaflight code move based on the main because main is working". The current code follows the same concept: 50 Hz PWM, 1500 stop, 1600 run, timed steps.

## Known Good Status Outputs

BMI088 working status example:

```text
devices: BMI088/SPI
gyro clock: 500 Hz
gyro rate: 500 Hz
accel rate: 500 Hz
```

Full sensors working before PMW:

```text
devices: BMI088/SPI, BMP388/I2C, BMM150/I2C, VL53L1X/RANGE
active: acc=1 mag=1 baro=1
sensors: acc_raw=34/-4/2035 acc=0.09/0.03/9.80 mag=-25.00/11.01/101.54 baro=99947.67Pa 22.83C alt_raw=115.31 alt=-0.03 vario=-0.32
aux: range=74mm status=0(valid)
```

Full sensors with PMW:

```text
devices: BMI088/SPI, BMP388/I2C, BMM150/I2C, VL53L1X/RANGE, PMW3901/FLOW
active: acc=1 mag=1 baro=1
sensors: acc_raw=22/46/2046 acc=0.09/0.28/9.77 mag=-36.26/-18.94/122.00 baro=100091.84Pa 24.25C alt_raw=103.18 alt=1.03 vario=-0.16
aux: range=47mm status=0(valid) pmw_id=0x49/0xB6 flow=0/0 frames=1040
```

Earlier broken PMW before wiring:

```text
pmw_id=0xFF/0xFF
```

That was because the PMW module was not wired yet.

## Known Bad Symptoms

ESP32-S3 watchdog reboot:

```text
rst:0x8 (TG1WDT_SYS_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)
```

Usually caused by blocking initialization or blocking sensor read in main/gyro path.

VL53 freezes:

- Sensors/status freezes when sonar is enabled.
- The problem happens when trying to read sonar data.
- VL53 is on different I2C bus than BMP/BMM/TCS.
- The working Arduino VL53 example uses GPIO42/GPIO41 and `Wire`.
- If reworking VL53, start from that example and preserve separate bus.

I2C error:

```text
[E][Wire.cpp:513] requestFrom(): i2cRead returned Error -1
```

Likely from missing/wrong I2C device, wrong bus, blocking read, or bus not wired/powered correctly.

Configurator data not changing:

- At one point BMI detected but 3D model did not move.
- Fix involved getting gyro and accel data into MSP raw IMU and ensuring accel active.
- Current status examples show `acc=1`, raw accel changing.

Baro altitude:

- BMP388 pressure can be jittery/drifty, especially indoors.
- Altitude depends on pressure baseline; some drift is normal.
- It is not GPS latitude. User once wrote "latitude altitude"; they meant altitude.

## Critical Notes For Next Codex

1. Do not delete firmware builds. The user was upset because old builds were deleted earlier. Keep timestamped builds in `firmware_builds`.

2. Before editing, inspect the real current files. This context may be slightly stale if the user changed code after it was generated.

3. Do not blindly trust `docs/drone-prototype.md`. It still contains stale info, for example TCS LED listed as GPIO19 while current target code uses GPIO18.

4. For VL53, do not read it blocking in the flight loop. Keep it isolated from gyro/accel/baro/mag timing.

5. For BMI088, remember it is SPI:

```text
SCK 9, MISO 10, MOSI 11, accel CS 7, gyro CS 8
```

6. For receiver wiring, receiver RX goes to FC TX GPIO37, receiver TX goes to FC RX GPIO38.

7. For DSHOT, current stable motor protocol is DSHOT300. Do not switch to another protocol unless explicitly testing.

8. For servo, MG90S 360 is continuous rotation. It cannot be commanded to exact angles by PWM alone. `1500 us` stop, `1600/1700 us` rotate. Timed steps are approximate only.

9. If the user asks for "web Betaflight", keep MSP compatibility high but do not claim this is official Betaflight firmware. It is ESP-FC with Betaflight-compatible MSP.

10. If enabling bidirectional DSHOT, preserve a known DSHOT300 non-bidirectional build so the user can recover.

## Suggested First Checks In A New Chat

Run these read-only checks first:

```powershell
cd "C:\Users\keena\OneDrive\Documents\PlatformIO\Projects\betaflight flight controller esp32\betaflight-fc"
git status --short
Get-Content -Raw platformio.ini
Get-Content -Raw lib\Espfc\src\Target\TargetESP32s3.h
Get-Content -Raw lib\Espfc\src\Sensor\AuxSensor.cpp
Get-Content -Raw lib\Espfc\src\Device\DroneProtoServo.cpp
```

If debugging a device:

```powershell
Get-Content -Raw src\main_drone_proto_clean.cpp
Get-Content -Raw lib\Espfc\src\Espfc.cpp
Get-Content -Raw lib\Espfc\src\Connect\Cli.cpp
Get-Content -Raw lib\Espfc\src\Connect\MspProcessor.cpp
```

Build:

```powershell
C:\Users\keena\.platformio\penv\Scripts\platformio.exe run -e drone_proto_esp32s3_clean_bmi088_gyro
```

After flashing, CLI checks:

```text
status
```

Useful CLI settings:

```text
set debug_mode RANGEFINDER
save
```

```text
set debug_mode RANGEFINDER_QUALITY
save
```

```text
set debug_mode ADC_INTERNAL
save
```

Servo CLI:

```text
servo
servo 1500
servo 1600
servo step
servo stop
servo off
```

TCS LED CLI:

```text
tcsled 1
tcsled 0
```

## Current Open/Unfinished Items

1. Restore/continue work in a fresh chat because Codex history restore failed.

2. Verify current `firmware_0x00_20260708_103937.bin` behavior on hardware.

3. Confirm TCS34725 is detected and color values appear in `status`/debug.

4. Confirm servo tab works in web Betaflight Configurator, not just CLI.

5. Decide whether `ESPFC_DRONE_PROTO_SERVO_AUTO_STEP` should remain enabled. It is useful for testing but may be annoying in normal FC operation.

6. Review `AuxSensor.cpp` debug macro delay behavior. It may be hiding 10 ms delays even with debug disabled.

7. Validate VL53 does not freeze the FC over longer runs.

8. Validate optical flow deltas change when moving the PMW3901 sensor over a textured surface.

9. Complete or validate CRSF 32-channel support. Current code adds subset channel parsing and increases internal CRSF channel count to 32, but the rest of the input stack may still need work.

10. Bidirectional DSHOT is not confirmed. Current safe motor setting is DSHOT300.

## Short Prompt To Paste With This File

Continue this project from the context in this file. First inspect the current files before editing. Do not delete firmware builds. Preserve the working DSHOT300/BMI088/BMP388/BMM150/VL53/PMW state. The immediate task is whatever I ask next, but use this context as the baseline.

