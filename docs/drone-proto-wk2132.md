# WK2132 / DFR0627 camera and UWB UART bridge

The Drone Prototype WK2132 framework uses a dedicated ESP32-S3 I2C controller
on GPIO47/GPIO48. It does not change UART0, the MTF02P UART, or the ELRS/CRSF
UART. WK2132 channel 1 is named `camera`; channel 2 is named `uwb`.

The defaults are compatible with the DFRobot DFR0627 circuit: 14.7456 MHz
oscillator and address switches IA1=1, IA0=1.

## Wiring

Use 3.3 V for the WK2132 and both UART devices so every logic level remains
safe for the ESP32-S3.

| Flight controller / signal | WK2132 pin |
| --- | --- |
| 3V3 | VCC |
| GND | GND |
| GPIO47 | MP0 / SDA |
| GPIO48 | MP2 / SCL |
| GPIO40 | RSTN |
| 3V3 | MD1 |
| GND | MD0 |
| 3V3 | MP3 / IA1 |
| 3V3 | MP1 / IA0 |

For a PCB based on the DFR0627 schematic, use its 5.1 kOhm pull-ups from SDA
and SCL to 3.3 V. Do not duplicate them if another fitted part already provides
pull-ups on this dedicated bus. Keep `RSTN` high with 10 kOhm and place 100 nF
from `RSTN` to ground; GPIO40 drives this active-low reset in the experimental
MTF02P build. The framework polls the FIFOs, so `IRQ` is optional and may be
left unconnected; if it is connected later, use the schematic's 5.1 kOhm
external pull-up. The DFR0627 also uses 10 kOhm pull-ups on RX1 and RX2 to
prevent idle inputs from floating.

For a bare WK2132, copy the oscillator circuit carefully: a 14.7456 MHz
crystal, 1 MOhm feedback resistor, and the crystal-appropriate load capacitors
(22 pF in the DFR0627 design). Keep the crystal, resistor, and capacitors close
to OSCI/OSCO with short traces. The experimental build assumes 14.7456 MHz. Change
`ESPFC_DRONE_PROTO_WK2132_OSCILLATOR_HZ` if the hardware uses another crystal.
The configured value must match the real crystal or both UART baud rates will
be wrong.

Cross each UART's TX and RX:

| WK2132 channel | Peripheral |
| --- | --- |
| TX1 | Camera RX |
| RX1 | Camera TX |
| TX2 | UWB RX |
| RX2 | UWB TX |

All boards must share ground. The WK2132's TX1/RX1 and TX2/RX2 pins are the new
UART pins; GPIO47/GPIO48 remain I2C only.

## Build and diagnostics

Build the opt-in environment:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e drone_proto_esp32s3_wk2132_experimental
```

After flashing that environment, enter the FC CLI and run:

```text
wk2132
```

`present=1`, `i2c_mode=1`, and `configured=1` for both ports indicate a valid
bridge configuration. `wk2132 retry` reinitializes the bridge after wiring is
corrected. `wk2132 clear camera` and `wk2132 clear uwb` explicitly clear a
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
