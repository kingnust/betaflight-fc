#!/usr/bin/env python3
"""Deterministic bench simulator for the Drone Prototype flight controls.

The model is intentionally small: it is not a replacement for an aerodynamic
simulator or a real flight test. It generates the same classes of measurements
used by the FC and verifies mode gating, bounded outputs, dropout handling, and
explicit recovery before hardware is available.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple


DT_S = 0.02
SENSOR_MAX_AGE_MS = 150
RANGE_MIN_MM = 80
RANGE_MAX_MM = 4000
MIN_FLOW_QUALITY = 30
MAX_TILT_RAD = math.radians(25.0)
PILOT_DEADBAND = 0.08
EXCESSIVE_VELOCITY_MPS = 2.0
EXCESSIVE_MOTION_TIME_MS = 100
PILOT_MAX_VELOCITY_MPS = 1.0
POSITION_GAIN = 0.8
POSITION_MAX_VELOCITY_MPS = 0.6
VELOCITY_TO_ANGLE = 0.14
MAX_CORRECTION_ANGLE_RAD = math.radians(8.0)
FLOW_FILTER_ALPHA = 0.30


class PositionHoldFault(str, Enum):
    OK = "OK"
    NO_ACCEL = "NO_ACCEL"
    NO_BARO = "NO_BARO"
    NO_RANGE = "NO_RANGE"
    RANGE_STALE = "RANGE_STALE"
    RANGE_INVALID = "RANGE_INVALID"
    NO_FLOW = "NO_FLOW"
    FLOW_STALE = "FLOW_STALE"
    FLOW_QUALITY = "FLOW_QUALITY"
    TILT = "TILT"


class PositionHoldRelease(str, Enum):
    NONE = "NONE"
    SENSOR = "SENSOR"
    EXCESSIVE_MOTION = "EXCESSIVE_MOTION"


@dataclass
class VehicleState:
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 1.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    roll_rate_rps: float = 0.0
    pitch_rate_rps: float = 0.0
    yaw_rate_rps: float = 0.0


@dataclass
class PilotInput:
    roll: float = 0.0
    pitch: float = 0.0
    throttle: float = 0.0
    altitude_hold_requested: bool = False
    position_hold_requested: bool = False


@dataclass
class SensorFaults:
    accel_missing: bool = False
    barometer_missing: bool = False
    range_missing: bool = False
    range_stale: bool = False
    range_override_mm: Optional[int] = None
    flow_missing: bool = False
    flow_stale: bool = False
    flow_quality: int = 120
    attitude_override_rad: Optional[Tuple[float, float]] = None


@dataclass
class SensorSample:
    gyro_x_rps: float
    gyro_y_rps: float
    gyro_z_rps: float
    baro_altitude_m: float
    baro_vario_mps: float
    range_mm: int
    flow_dx: float
    flow_dy: float
    flow_quality: int
    accel_present: bool
    baro_present: bool
    range_present: bool
    flow_present: bool
    range_updated_ms: int
    flow_updated_ms: int
    estimated_roll_rad: float
    estimated_pitch_rad: float


@dataclass
class PositionHoldOutput:
    active: bool
    healthy: bool
    fault: PositionHoldFault
    release: PositionHoldRelease
    roll_setpoint_rad: float
    pitch_setpoint_rad: float
    estimated_x_m: float
    estimated_y_m: float
    estimated_vx_mps: float
    estimated_vy_mps: float


@dataclass
class TraceRow:
    time_s: float
    scenario: str
    true_x_m: float
    true_y_m: float
    true_z_m: float
    true_vx_mps: float
    true_vy_mps: float
    true_vz_mps: float
    gyro_x_rps: float
    gyro_y_rps: float
    gyro_z_rps: float
    baro_altitude_m: float
    baro_vario_mps: float
    range_mm: int
    flow_dx: float
    flow_dy: float
    flow_quality: int
    altitude_hold_active: int
    position_hold_requested: int
    position_hold_active: int
    position_hold_healthy: int
    position_hold_fault: str
    position_hold_release: str
    estimated_x_m: float
    estimated_y_m: float
    estimated_vx_mps: float
    estimated_vy_mps: float
    roll_setpoint_deg: float
    pitch_setpoint_deg: float
    climb_rate_setpoint_mps: float
    vertical_accel_command_mps2: float


@dataclass
class ScenarioSummary:
    scenario: str
    samples: int
    all_outputs_finite: bool
    final_altitude_m: float
    final_vertical_speed_mps: float
    final_position_error_m: float
    final_horizontal_speed_mps: float
    maximum_correction_angle_deg: float
    active_samples: int
    inactive_samples: int
    recovered_after_release: bool
    release_reasons: List[str]


class SyntheticSensors:
    def __init__(self, seed: int = 4147) -> None:
        self._rng = random.Random(seed)
        self._last_range_ms = 1
        self._last_flow_ms = 1
        self._baro_bias_m = 0.0

    def sample(
        self,
        vehicle: VehicleState,
        now_ms: int,
        faults: SensorFaults,
    ) -> SensorSample:
        self._baro_bias_m += self._rng.gauss(0.0, 0.00003)
        baro_altitude = vehicle.z_m + self._baro_bias_m + self._rng.gauss(0.0, 0.008)
        baro_vario = vehicle.vz_mps + self._rng.gauss(0.0, 0.025)

        range_mm = max(0, int(round(vehicle.z_m * 1000.0 + self._rng.gauss(0.0, 3.0))))
        if faults.range_override_mm is not None:
            range_mm = faults.range_override_mm
        if not faults.range_stale:
            self._last_range_ms = now_ms

        height_m = max(vehicle.z_m, 0.08)
        flow_dx = vehicle.vx_mps / (height_m * 0.01) + self._rng.gauss(0.0, 0.20)
        flow_dy = vehicle.vy_mps / (height_m * 0.01) + self._rng.gauss(0.0, 0.20)
        if not faults.flow_stale:
            self._last_flow_ms = now_ms

        roll = vehicle.roll_rad
        pitch = vehicle.pitch_rad
        if faults.attitude_override_rad is not None:
            roll, pitch = faults.attitude_override_rad

        return SensorSample(
            gyro_x_rps=vehicle.roll_rate_rps + self._rng.gauss(0.0, 0.002),
            gyro_y_rps=vehicle.pitch_rate_rps + self._rng.gauss(0.0, 0.002),
            gyro_z_rps=vehicle.yaw_rate_rps + self._rng.gauss(0.0, 0.002),
            baro_altitude_m=baro_altitude,
            baro_vario_mps=baro_vario,
            range_mm=range_mm,
            flow_dx=flow_dx,
            flow_dy=flow_dy,
            flow_quality=faults.flow_quality,
            accel_present=not faults.accel_missing,
            baro_present=not faults.barometer_missing,
            range_present=not faults.range_missing,
            flow_present=not faults.flow_missing,
            range_updated_ms=self._last_range_ms,
            flow_updated_ms=self._last_flow_ms,
            estimated_roll_rad=roll,
            estimated_pitch_rad=pitch,
        )


class AltitudeHoldController:
    """Vertical-speed controller matching the FC mode's stick semantics."""

    def __init__(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0

    @staticmethod
    def climb_rate_setpoint(throttle: float) -> float:
        throttle = max(-1.0, min(1.0, throttle))
        if abs(throttle) <= 0.10:
            throttle = 0.0
        elif throttle > 0.0:
            throttle = (throttle - 0.10) / 0.90
        else:
            throttle = (throttle + 0.10) / 0.90
        return throttle * (4.0 if throttle >= 0.0 else 2.0)

    def update(self, throttle: float, measured_vario_mps: float, active: bool) -> Tuple[float, float]:
        desired_vz = self.climb_rate_setpoint(throttle) if active else 0.0
        if not active:
            self._integral *= 0.95
            self._last_error = 0.0
            return desired_vz, max(-3.0, min(3.0, throttle * 3.0))

        error = desired_vz - measured_vario_mps
        self._integral = max(-2.0, min(2.0, self._integral + error * DT_S))
        derivative = (error - self._last_error) / DT_S
        self._last_error = error
        command = 1.35 * error + 0.55 * self._integral + 0.03 * derivative
        return desired_vz, max(-4.0, min(4.0, command))


class OpticalFlowPositionHoldController:
    def __init__(self) -> None:
        self.active = False
        self.release_latched = False
        self.release = PositionHoldRelease.NONE
        self._was_requested = False
        self._excessive_motion_since_ms: Optional[int] = None
        self._filtered_velocity = [0.0, 0.0]
        self._position = [0.0, 0.0]
        self._target = [0.0, 0.0]
        self._last_flow_update_ms = 0

    @staticmethod
    def _fresh(updated_ms: int, now_ms: int) -> bool:
        return updated_ms != 0 and (now_ms - updated_ms) <= SENSOR_MAX_AGE_MS

    @staticmethod
    def _deadband(value: float) -> float:
        if abs(value) <= PILOT_DEADBAND:
            return 0.0
        magnitude = (abs(value) - PILOT_DEADBAND) / (1.0 - PILOT_DEADBAND)
        return math.copysign(min(magnitude, 1.0), value)

    def sensor_fault(self, sensor: SensorSample, now_ms: int) -> PositionHoldFault:
        if not sensor.accel_present:
            return PositionHoldFault.NO_ACCEL
        if not sensor.baro_present:
            return PositionHoldFault.NO_BARO
        if not sensor.range_present:
            return PositionHoldFault.NO_RANGE
        if not self._fresh(sensor.range_updated_ms, now_ms):
            return PositionHoldFault.RANGE_STALE
        if sensor.range_mm < RANGE_MIN_MM or sensor.range_mm > RANGE_MAX_MM:
            return PositionHoldFault.RANGE_INVALID
        if not sensor.flow_present:
            return PositionHoldFault.NO_FLOW
        if not self._fresh(sensor.flow_updated_ms, now_ms):
            return PositionHoldFault.FLOW_STALE
        if sensor.flow_quality < MIN_FLOW_QUALITY:
            return PositionHoldFault.FLOW_QUALITY
        if max(abs(sensor.estimated_roll_rad), abs(sensor.estimated_pitch_rad)) > MAX_TILT_RAD:
            return PositionHoldFault.TILT
        return PositionHoldFault.OK

    def _update_flow(self, sensor: SensorSample, yaw_rad: float) -> None:
        if sensor.flow_updated_ms == self._last_flow_update_ms:
            return
        dt = DT_S
        if self._last_flow_update_ms:
            dt = max(0.005, min(0.10, (sensor.flow_updated_ms - self._last_flow_update_ms) * 0.001))
        self._last_flow_update_ms = sensor.flow_updated_ms

        height_m = sensor.range_mm * 0.001
        measured_forward = sensor.flow_dx * height_m * 0.01
        measured_right = sensor.flow_dy * height_m * 0.01
        self._filtered_velocity[0] += (measured_forward - self._filtered_velocity[0]) * FLOW_FILTER_ALPHA
        self._filtered_velocity[1] += (measured_right - self._filtered_velocity[1]) * FLOW_FILTER_ALPHA

        cosine = math.cos(yaw_rad)
        sine = math.sin(yaw_rad)
        earth_vx = cosine * self._filtered_velocity[0] - sine * self._filtered_velocity[1]
        earth_vy = sine * self._filtered_velocity[0] + cosine * self._filtered_velocity[1]
        self._position[0] += earth_vx * dt
        self._position[1] += earth_vy * dt

    def update(
        self,
        sensor: SensorSample,
        pilot: PilotInput,
        yaw_rad: float,
        now_ms: int,
    ) -> PositionHoldOutput:
        requested = pilot.position_hold_requested
        if not requested:
            self.release_latched = False
            self.release = PositionHoldRelease.NONE
            self._excessive_motion_since_ms = None

        fault = self.sensor_fault(sensor, now_ms)
        healthy = fault == PositionHoldFault.OK
        if healthy:
            self._update_flow(sensor, yaw_rad)

        speed = math.hypot(*self._filtered_velocity)
        if self.active and speed > EXCESSIVE_VELOCITY_MPS:
            if self._excessive_motion_since_ms is None:
                self._excessive_motion_since_ms = now_ms
        else:
            self._excessive_motion_since_ms = None

        if self.active and not healthy:
            self.release_latched = True
            self.release = PositionHoldRelease.SENSOR
        elif (
            self.active
            and self._excessive_motion_since_ms is not None
            and now_ms - self._excessive_motion_since_ms >= EXCESSIVE_MOTION_TIME_MS
        ):
            self.release_latched = True
            self.release = PositionHoldRelease.EXCESSIVE_MOTION

        if not requested or not healthy or self.release_latched:
            self.active = False
            self._was_requested = requested
            return self._output(healthy, fault, 0.0, 0.0)

        if not self._was_requested or not self.active:
            self._target[:] = self._position
        self._was_requested = True
        self.active = True

        pilot_forward = self._deadband(pilot.pitch)
        pilot_right = self._deadband(pilot.roll)
        if pilot_forward or pilot_right:
            desired_forward = pilot_forward * PILOT_MAX_VELOCITY_MPS
            desired_right = pilot_right * PILOT_MAX_VELOCITY_MPS
            self._target[:] = self._position
        else:
            desired_earth_x = max(
                -POSITION_MAX_VELOCITY_MPS,
                min(POSITION_MAX_VELOCITY_MPS, (self._target[0] - self._position[0]) * POSITION_GAIN),
            )
            desired_earth_y = max(
                -POSITION_MAX_VELOCITY_MPS,
                min(POSITION_MAX_VELOCITY_MPS, (self._target[1] - self._position[1]) * POSITION_GAIN),
            )
            cosine = math.cos(yaw_rad)
            sine = math.sin(yaw_rad)
            desired_forward = cosine * desired_earth_x + sine * desired_earth_y
            desired_right = -sine * desired_earth_x + cosine * desired_earth_y

        pitch = (desired_forward - self._filtered_velocity[0]) * VELOCITY_TO_ANGLE
        roll = (desired_right - self._filtered_velocity[1]) * VELOCITY_TO_ANGLE
        pitch = max(-MAX_CORRECTION_ANGLE_RAD, min(MAX_CORRECTION_ANGLE_RAD, pitch))
        roll = max(-MAX_CORRECTION_ANGLE_RAD, min(MAX_CORRECTION_ANGLE_RAD, roll))
        return self._output(healthy, fault, roll, pitch)

    def _output(
        self,
        healthy: bool,
        fault: PositionHoldFault,
        roll: float,
        pitch: float,
    ) -> PositionHoldOutput:
        return PositionHoldOutput(
            active=self.active,
            healthy=healthy,
            fault=fault,
            release=self.release,
            roll_setpoint_rad=roll,
            pitch_setpoint_rad=pitch,
            estimated_x_m=self._position[0],
            estimated_y_m=self._position[1],
            estimated_vx_mps=self._filtered_velocity[0],
            estimated_vy_mps=self._filtered_velocity[1],
        )


def scenario_state(name: str, time_s: float) -> Tuple[PilotInput, SensorFaults]:
    pilot = PilotInput()
    faults = SensorFaults()

    if name == "altitude_hold":
        pilot.altitude_hold_requested = True
    else:
        pilot.position_hold_requested = True

    if name == "position_hold" and 4.0 <= time_s < 4.8:
        pilot.pitch = 0.35
    elif name == "flow_dropout":
        faults.flow_stale = 2.0 <= time_s < 2.45
        pilot.position_hold_requested = not (3.0 <= time_s < 3.25)
    elif name == "barometer_dropout":
        faults.barometer_missing = 2.0 <= time_s < 2.40
        pilot.position_hold_requested = not (3.0 <= time_s < 3.25)
    elif name == "invalid_range":
        if 2.0 <= time_s < 2.35:
            faults.range_override_mm = 33
        pilot.position_hold_requested = not (3.0 <= time_s < 3.25)
    elif name == "low_flow_quality":
        faults.flow_quality = 5 if 2.0 <= time_s < 2.35 else 120
        pilot.position_hold_requested = not (3.0 <= time_s < 3.25)
    elif name == "excessive_motion":
        pilot.position_hold_requested = not (3.0 <= time_s < 3.25)
    elif name not in {"altitude_hold", "position_hold"}:
        raise ValueError(f"unknown scenario: {name}")
    return pilot, faults


SCENARIOS: Tuple[str, ...] = (
    "altitude_hold",
    "position_hold",
    "flow_dropout",
    "barometer_dropout",
    "invalid_range",
    "low_flow_quality",
    "excessive_motion",
)


class FlightSimulation:
    def __init__(self, scenario: str, duration_s: float = 6.0, seed: int = 4147) -> None:
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario: {scenario}")
        self.scenario = scenario
        self.duration_s = duration_s
        self.vehicle = VehicleState()
        self.sensors = SyntheticSensors(seed)
        self.altitude = AltitudeHoldController()
        self.position = OpticalFlowPositionHoldController()
        self._events_applied: set[str] = set()

    def _apply_disturbances(self, time_s: float) -> None:
        if time_s >= 1.0 and "initial_gust" not in self._events_applied:
            self.vehicle.vx_mps += 0.55
            self.vehicle.vy_mps -= 0.35
            self.vehicle.vz_mps += 0.55
            self._events_applied.add("initial_gust")
        if self.scenario == "excessive_motion" and time_s >= 2.0 and "fast_gust" not in self._events_applied:
            self.vehicle.vx_mps = 2.8
            self._events_applied.add("fast_gust")

    def run(self) -> Tuple[List[TraceRow], ScenarioSummary]:
        rows: List[TraceRow] = []
        release_reasons: List[str] = []
        released_once = False
        recovered_after_release = False
        maximum_angle_deg = 0.0
        steps = int(round(self.duration_s / DT_S))

        for step in range(steps):
            time_s = (step + 1) * DT_S
            now_ms = int(round(time_s * 1000.0))
            self._apply_disturbances(time_s)
            pilot, faults = scenario_state(self.scenario, time_s)
            sensor = self.sensors.sample(self.vehicle, now_ms, faults)
            position = self.position.update(sensor, pilot, self.vehicle.yaw_rad, now_ms)

            if position.release != PositionHoldRelease.NONE and position.release.value not in release_reasons:
                release_reasons.append(position.release.value)
                released_once = True
            if released_once and position.active:
                recovered_after_release = True

            vertical_hold_active = pilot.altitude_hold_requested or position.active
            climb_rate, vertical_accel = self.altitude.update(
                pilot.throttle,
                sensor.baro_vario_mps,
                vertical_hold_active and sensor.baro_present,
            )

            roll_setpoint = position.roll_setpoint_rad if position.active else pilot.roll * math.radians(25.0)
            pitch_setpoint = position.pitch_setpoint_rad if position.active else pilot.pitch * math.radians(25.0)
            maximum_angle_deg = max(
                maximum_angle_deg,
                abs(math.degrees(roll_setpoint)),
                abs(math.degrees(pitch_setpoint)),
            )
            self._advance_vehicle(roll_setpoint, pitch_setpoint, vertical_accel)

            rows.append(
                TraceRow(
                    time_s=time_s,
                    scenario=self.scenario,
                    true_x_m=self.vehicle.x_m,
                    true_y_m=self.vehicle.y_m,
                    true_z_m=self.vehicle.z_m,
                    true_vx_mps=self.vehicle.vx_mps,
                    true_vy_mps=self.vehicle.vy_mps,
                    true_vz_mps=self.vehicle.vz_mps,
                    gyro_x_rps=sensor.gyro_x_rps,
                    gyro_y_rps=sensor.gyro_y_rps,
                    gyro_z_rps=sensor.gyro_z_rps,
                    baro_altitude_m=sensor.baro_altitude_m,
                    baro_vario_mps=sensor.baro_vario_mps,
                    range_mm=sensor.range_mm,
                    flow_dx=sensor.flow_dx,
                    flow_dy=sensor.flow_dy,
                    flow_quality=sensor.flow_quality,
                    altitude_hold_active=int(vertical_hold_active),
                    position_hold_requested=int(pilot.position_hold_requested),
                    position_hold_active=int(position.active),
                    position_hold_healthy=int(position.healthy),
                    position_hold_fault=position.fault.value,
                    position_hold_release=position.release.value,
                    estimated_x_m=position.estimated_x_m,
                    estimated_y_m=position.estimated_y_m,
                    estimated_vx_mps=position.estimated_vx_mps,
                    estimated_vy_mps=position.estimated_vy_mps,
                    roll_setpoint_deg=math.degrees(roll_setpoint),
                    pitch_setpoint_deg=math.degrees(pitch_setpoint),
                    climb_rate_setpoint_mps=climb_rate,
                    vertical_accel_command_mps2=vertical_accel,
                )
            )

        all_values: Iterable[float] = (
            value
            for row in rows
            for value in (
                row.true_x_m,
                row.true_y_m,
                row.true_z_m,
                row.true_vx_mps,
                row.true_vy_mps,
                row.true_vz_mps,
                row.roll_setpoint_deg,
                row.pitch_setpoint_deg,
                row.vertical_accel_command_mps2,
            )
        )
        summary = ScenarioSummary(
            scenario=self.scenario,
            samples=len(rows),
            all_outputs_finite=all(math.isfinite(value) for value in all_values),
            final_altitude_m=self.vehicle.z_m,
            final_vertical_speed_mps=self.vehicle.vz_mps,
            final_position_error_m=math.hypot(self.vehicle.x_m, self.vehicle.y_m),
            final_horizontal_speed_mps=math.hypot(self.vehicle.vx_mps, self.vehicle.vy_mps),
            maximum_correction_angle_deg=maximum_angle_deg,
            active_samples=sum(row.position_hold_active for row in rows),
            inactive_samples=sum(1 - row.position_hold_active for row in rows),
            recovered_after_release=recovered_after_release,
            release_reasons=release_reasons,
        )
        return rows, summary

    def _advance_vehicle(self, roll_setpoint: float, pitch_setpoint: float, vertical_accel: float) -> None:
        attitude_time_constant_s = 0.16
        self.vehicle.roll_rate_rps = (roll_setpoint - self.vehicle.roll_rad) / attitude_time_constant_s
        self.vehicle.pitch_rate_rps = (pitch_setpoint - self.vehicle.pitch_rad) / attitude_time_constant_s
        self.vehicle.roll_rad += self.vehicle.roll_rate_rps * DT_S
        self.vehicle.pitch_rad += self.vehicle.pitch_rate_rps * DT_S

        horizontal_drag = 0.55
        ax = 9.81 * self.vehicle.pitch_rad - horizontal_drag * self.vehicle.vx_mps
        ay = 9.81 * self.vehicle.roll_rad - horizontal_drag * self.vehicle.vy_mps
        az = vertical_accel - 0.18 * self.vehicle.vz_mps
        self.vehicle.vx_mps += ax * DT_S
        self.vehicle.vy_mps += ay * DT_S
        self.vehicle.vz_mps += az * DT_S
        self.vehicle.x_m += self.vehicle.vx_mps * DT_S
        self.vehicle.y_m += self.vehicle.vy_mps * DT_S
        self.vehicle.z_m = max(0.05, self.vehicle.z_m + self.vehicle.vz_mps * DT_S)


def write_trace(path: Path, rows: Sequence[TraceRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def run_scenarios(names: Sequence[str], duration_s: float, output: Optional[Path]) -> List[ScenarioSummary]:
    summaries: List[ScenarioSummary] = []
    for name in names:
        rows, summary = FlightSimulation(name, duration_s=duration_s).run()
        summaries.append(summary)
        if output is not None:
            write_trace(output / f"{name}.csv", rows)
    if output is not None:
        output.mkdir(parents=True, exist_ok=True)
        with (output / "summary.json").open("w", encoding="utf-8") as stream:
            json.dump([asdict(summary) for summary in summaries], stream, indent=2)
            stream.write("\n")
    return summaries


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=("all",) + SCENARIOS,
        default="all",
        help="scenario to run (default: all)",
    )
    parser.add_argument("--duration", type=float, default=6.0, help="simulation duration in seconds")
    parser.add_argument("--output", type=Path, help="optional directory for CSV traces and summary.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    names = SCENARIOS if args.scenario == "all" else (args.scenario,)
    summaries = run_scenarios(names, args.duration, args.output)
    print(json.dumps([asdict(summary) for summary in summaries], indent=2))
    return 0 if all(summary.all_outputs_finite for summary in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
