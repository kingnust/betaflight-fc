# Drone Prototype ESP32-S3 Target

This target maps the sensors used by the Arduino Drone Prototype examples onto ESP-FC so they can be detected through the normal ESP-FC sensor stack and exposed through Betaflight Configurator-compatible MSP/CLI settings.

## Build target

Use the PlatformIO environment:

```text
drone_proto_esp32s3
```

This environment defines `ESPFC_TARGET_DRONE_PROTO` and uses the `4d_systems_esp32s3_gen4_r8n16` board from the Drone Prototype project.

## Supported prototype sensors

| Prototype sensor | ESP-FC role | Bus | Pins | Configurator/CLI device |
| --- | --- | --- | --- | --- |
| BMI088 accel | Accelerometer | SPI | SCK 9, MISO 10, MOSI 11, CS 7 | `gyro_dev BMI088`, `accel_dev BMI088` |
| BMI088 gyro | Gyroscope | SPI | SCK 9, MISO 10, MOSI 11, CS 8 | `gyro_dev BMI088`, `accel_dev BMI088` |
| BMP388/BMP390 | Barometer | I2C | SDA 17, SCL 16 | `baro_dev BMP388` or `AUTO` |
| BMM150 | Magnetometer | I2C | SDA 17, SCL 16 | `mag_dev BMM150` or `AUTO` |
| ELRS/CRSF receiver | Serial RX | UART2 | RX 38, TX 37 | Receiver provider CRSF |
| VL53L1X | Rangefinder readout | I2C1 | SDA 42, SCL 41 | MSP sensor bit + debug readout |
| PMW3901 | Optical-flow readout | SPI | SCK 9, MISO 10, MOSI 11, CS 40 | Debug/CLI readout |
| TCS34725 | Color readout | I2C | SDA 17, SCL 16, LED 19 | Debug/CLI readout |

The target defaults enable CRSF, I2C at 400 kHz, barometer auto-detection, and magnetometer auto-detection. The flight attitude fusion remains ESP-FC's normal fusion path; the Arduino Madgwick/Reefwing code is not used here.

The VL53L1X, PMW3901, and TCS34725 are integrated as auxiliary readout sensors. VL53L1X is exposed through the MSP sonar/rangefinder status bit. PMW3901 optical flow and TCS34725 color are available through CLI `status` and debug values, but this ESP-FC tree does not have a Betaflight navigation stack that consumes optical flow or color data for flight control.

## Pin map

```text
SPI SCK       GPIO9
SPI MISO      GPIO10
SPI MOSI      GPIO11
BMI088 accel  GPIO7  (pin_spi_cs_0)
BMI088 gyro   GPIO8  (pin_spi_cs_1 in this target)

I2C0 SDA      GPIO17
I2C0 SCL      GPIO16
VL53 SDA      GPIO42
VL53 SCL      GPIO41
TCS LED       GPIO19

CRSF RX       GPIO38
CRSF TX       GPIO37

PMW3901 CS    GPIO40

Motor 1 / M1  GPIO12
Motor 2 / M2  GPIO13
Motor 3 / M3  GPIO14
Motor 4 / M4  GPIO15
```

`pin_spi_cs_1` is named as a barometer CS in the older generic target, but in this Drone Prototype preset it is intentionally used as the second BMI088 chip-select. The BMP388/BMP390 barometer is I2C-only in this target.

The code intentionally follows the prototype examples for the auxiliary sensor pins. If a schematic revision disagrees with these values, update the schematic or adjust the target macros in `TargetESP32s3.h` before flashing.

## Useful CLI checks

On a fresh flash the target defaults should already match this wiring. If old EEPROM settings are still stored, run `defaults`, reboot, then confirm:

```text
set gyro_dev AUTO
set accel_dev AUTO
set mag_dev AUTO
set baro_dev AUTO
set i2c_speed 400
set pin_spi_0_sck 9
set pin_spi_0_miso 10
set pin_spi_0_mosi 11
set pin_spi_cs_0 7
set pin_spi_cs_1 8
set pin_i2c_sda 17
set pin_i2c_scl 16
set pin_serial_2_rx 38
set pin_serial_2_tx 37
set pin_output_0 12
set pin_output_1 13
set pin_output_2 14
set pin_output_3 15
save
```

Set the serial receiver provider to CRSF/ELRS in Betaflight Configurator's receiver settings if an older saved configuration still shows SBUS.

## Auxiliary sensor readout

Use CLI `status` to see detected auxiliary devices and latest values. In Betaflight Configurator's Sensors/Status area, VL53L1X should appear through the rangefinder/sonar bit when detected.

Use these debug modes in Configurator:

```text
set debug_mode RANGEFINDER
```

Debug values are range in mm, range status, PMW3901 delta X, and PMW3901 delta Y.

```text
set debug_mode RANGEFINDER_QUALITY
```

Debug values are range status, signal, and ambient.

```text
set debug_mode ADC_INTERNAL
```

Debug values are TCS34725 red, green, blue, and clear raw channels.
