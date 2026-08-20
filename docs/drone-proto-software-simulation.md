# Drone Prototype Software Simulation

This host-only test system exercises flight-control and input-safety behavior
before the final PCB, sensors, motors, and airframe are available. It does not
flash hardware and it never drives motor outputs.

## Run everything

From the `betaflight-fc` repository in PowerShell:

```powershell
.\tools\fc_simulator\run_all.cmd
```

The runner performs three steps:

1. Runs deterministic altitude-hold and optical-flow position-hold assertions.
2. Compiles and runs the production command router and receiver-safety policy.
3. Writes CSV traces and `summary.json` to `tools\fc_simulator\output`.

Use `-SkipTraces` when only pass/fail tests are needed:

```powershell
.\tools\fc_simulator\run_all.cmd -SkipTraces
```

## Flight simulator

`simulator.py` models a small quadrotor at 50 Hz and generates deterministic:

- gyroscope rates;
- barometer altitude and vertical speed;
- downward range in millimetres;
- height-scaled optical-flow measurements and quality;
- sensor noise and slow barometer bias.

The altitude controller follows the FC's current behavior: centered throttle
requests zero vertical speed, while throttle movement requests climb or descent.
The optical-flow controller uses the FC's current freshness, range, quality,
tilt, speed, deadband, velocity, and correction-angle limits.

Available scenarios:

| Scenario | Purpose |
| --- | --- |
| `altitude_hold` | Reject a vertical gust using barometer vertical speed. |
| `position_hold` | Stop an XY gust, accept pilot movement, then hold again. |
| `flow_dropout` | Reject flow older than 150 ms and require a mode cycle. |
| `barometer_dropout` | Release position hold when barometer health is lost. |
| `invalid_range` | Reject a fresh but impossible 33 mm range measurement. |
| `low_flow_quality` | Reject optical flow below the quality threshold. |
| `excessive_motion` | Latch a release after velocity exceeds 2 m/s for 100 ms. |

Run one scenario without writing files:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" `
  .\tools\fc_simulator\simulator.py --scenario flow_dropout
```

Generate traces for one scenario:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" `
  .\tools\fc_simulator\simulator.py `
  --scenario position_hold `
  --output .\tools\fc_simulator\output
```

The CSV columns include true vehicle state, every synthetic sensor value,
health and release states, estimated XY velocity/position, angle corrections,
and vertical controller commands. The files can be graphed directly in Excel.

## Automated safety coverage

The `native_drone_proto` PlatformIO environment compiles hardware-independent
production code rather than a second mock implementation. It checks:

- one-millisecond arm and trainer glitches are ignored;
- armed RadioMaster control blocks trainer takeover without disarming;
- trainer entry requires a complete fresh phone frame and arm-low period;
- phone loss exits trainer ownership and prevents inherited radio arming;
- trainer recovery requires both arm sources low;
- direct Wi-Fi control has priority and cannot silently restore trainer;
- task commands expire after their deadline, including timer wraparound;
- receiver loss, receiver failsafe, missing frames, and invalid channels block arming;
- receiver readiness returns only after every receiver fault clears.

Run only the production safety tests:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" `
  test -e native_drone_proto
```

## Limits

This is a deterministic control and safety test bench, not a high-fidelity
aerodynamic model. It can catch sign errors, unsafe mode transitions, stale or
invalid sensor handling, unbounded outputs, and recovery mistakes. It cannot
validate propeller thrust, vibration, motor balance, real sensor latency,
airframe resonance, or final PID gains. Those still require staged bench tests
and guarded flight tests after assembly.
