# WK2132 camera and UWB UART bridge

This configuration matches `Schematic_Drone_PCB_2026-07-28.pdf`. The WK2132
shares the FC sensor I2C bus on GPIO17/GPIO16. It uses the already initialized
`EspWire` instance and never starts, reconfigures, or stops a second I2C
controller on those pins.

WK2132 channel 1 is `uwb`; channel 2 is `camera`. The PCB uses a 14.7456 MHz
crystal and address straps IA1=0, IA0=1. The resulting register/FIFO address
pairs are 0x30/0x31 for UWB and 0x32/0x33 for camera.

## Wiring

Use 3.3 V for the WK2132 and both UART devices so every logic level remains
safe for the ESP32-S3.

| Flight controller / signal | WK2132 pin |
| --- | --- |
| 3V3 | VCC |
| GND | GND |
| GPIO17 / I2C1_SDA | MP0 / SDA |
| GPIO16 / I2C1_SCL | MP2 / SCL |
| RC reset only | RSTN |
| GPIO6 | IRQ |
| 3V3 | MD1 |
| GND | MD0 |
| GND | MP3 / IA1 |
| 3V3 | MP1 / IA0 |

The PCB has one 4.7 kOhm pull-up on each shared I2C line. Do not fit another
strong pull-up pair on attached sensor modules. `RSTN` uses a 10 kOhm pull-up
and 100 nF capacitor; firmware therefore leaves reset unassigned and uses the
WK2132 global software reset after detection. `IRQ` is wired to GPIO6 with a
5.1 kOhm external pull-up. The current transport polls the FIFOs, so GPIO6 is
configured as an input but no interrupt handler is attached.

The oscillator is a 14.7456 MHz crystal with a 1 MOhm feedback resistor and
18 pF load capacitors. Keep these parts close to OSCI/OSCO. The configured
oscillator value must match the fitted crystal or both UART baud rates will be
wrong.

Cross each UART's TX and RX:

| WK2132 channel | Peripheral |
| --- | --- |
| TX1 | UWB RX |
| RX1 | UWB TX |
| TX2 | Camera RX |
| RX2 | Camera TX |

All boards must share ground. The camera connector is powered from 5 V, while
the BU4 UWB connector is powered from 3.3 V as shown in the July 28 schematic.
Confirm that both peripherals use 3.3 V UART logic.

## Build and diagnostics

Build the opt-in environment:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e drone_proto_esp32s3_wk2132_experimental
```

After flashing that environment, enter the FC CLI and run:

```text
wk2132
```

`present=1`, `i2c_mode=1`, `shared_bus=1`, camera `ch=2`, UWB `ch=1`, and
`configured=1` for both ports indicate a valid bridge configuration.
`wk2132 retry` reinitializes only the WK2132 and does not restart the shared
I2C bus. `wk2132 clear camera` and `wk2132 clear uwb` explicitly clear a
port's FIFOs.

## Code access

The ports implement the FC's `SerialDevice` interface:

```cpp
#include "Device/DroneProtoWk2132.hpp"

auto& camera = Espfc::Device::DroneProtoWk2132::cameraPort();
auto& uwb = Espfc::Device::DroneProtoWk2132::uwbPort();
```

They support `available()`, `read()`, `readMany()`, `write()`,
`availableForWrite()`, and a bounded `flush()`. The framework only provides
reliable UART transport; camera message parsing and UWB positioning protocols
remain separate consumers so they cannot affect flight controls until they are
explicitly integrated.
