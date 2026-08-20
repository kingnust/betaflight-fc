#!/usr/bin/env python3
"""Validate a DLOC1 map and optionally render QR landmark PNG files."""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path


ID_PATTERN = re.compile(r"^[A-Za-z0-9_-]{1,15}$")


@dataclass(frozen=True)
class Landmark:
    marker_id: str
    x_m: float
    y_m: float
    z_m: float
    edge_yaw_deg: float
    size_m: float

    @property
    def payload(self) -> str:
        return (
            f"DLOC1,{self.marker_id},{self.x_m:.3f},{self.y_m:.3f},"
            f"{self.z_m:.3f},{self.edge_yaw_deg:.1f},{self.size_m:.3f}"
        )


def finite_float(row: dict[str, str], name: str, line_number: int) -> float:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"line {line_number}: invalid {name}") from error
    if not math.isfinite(value):
        raise ValueError(f"line {line_number}: {name} must be finite")
    return value


def read_landmarks(path: Path) -> list[Landmark]:
    landmarks: list[Landmark] = []
    identifiers: set[str] = set()
    with path.open("r", encoding="ascii", newline="") as source:
        reader = csv.DictReader(source)
        expected = {"id", "x_m", "y_m", "z_m", "edge_yaw_deg", "size_m"}
        if set(reader.fieldnames or []) != expected:
            raise ValueError(f"CSV columns must be: {', '.join(sorted(expected))}")
        for line_number, row in enumerate(reader, start=2):
            marker_id = (row.get("id") or "").strip()
            if not ID_PATTERN.fullmatch(marker_id):
                raise ValueError(f"line {line_number}: invalid id {marker_id!r}")
            if marker_id in identifiers:
                raise ValueError(f"line {line_number}: duplicate id {marker_id!r}")
            landmark = Landmark(
                marker_id,
                finite_float(row, "x_m", line_number),
                finite_float(row, "y_m", line_number),
                finite_float(row, "z_m", line_number),
                finite_float(row, "edge_yaw_deg", line_number),
                finite_float(row, "size_m", line_number),
            )
            if abs(landmark.x_m) > 1000 or abs(landmark.y_m) > 1000:
                raise ValueError(f"line {line_number}: x/y must be within 1000 m")
            if abs(landmark.z_m) > 100:
                raise ValueError(f"line {line_number}: z must be within 100 m")
            if abs(landmark.edge_yaw_deg) > 360:
                raise ValueError(f"line {line_number}: edge yaw must be within 360 degrees")
            if not 0.03 <= landmark.size_m <= 2.0:
                raise ValueError(f"line {line_number}: size must be 0.03 to 2.0 m")
            if len(landmark.payload.encode("ascii")) > 96:
                raise ValueError(f"line {line_number}: encoded payload exceeds 96 bytes")
            identifiers.add(marker_id)
            landmarks.append(landmark)
    if not landmarks:
        raise ValueError("landmark map is empty")
    return landmarks


def render_pngs(landmarks: list[Landmark], output: Path) -> None:
    try:
        import qrcode
    except ImportError as error:
        raise RuntimeError(
            'PNG output needs the optional package: python -m pip install "qrcode[pil]"'
        ) from error

    for landmark in landmarks:
        image = qrcode.make(landmark.payload, error_correction=qrcode.constants.ERROR_CORRECT_M)
        image.save(output / f"{landmark.marker_id}.png")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="landmark CSV file")
    parser.add_argument("--output", type=Path, help="output directory")
    parser.add_argument("--png", action="store_true", help="render PNG QR files")
    args = parser.parse_args()

    landmarks = read_landmarks(args.csv)
    output = args.output or args.csv.parent / "generated_landmarks"
    output.mkdir(parents=True, exist_ok=True)
    payload_file = output / "payloads.txt"
    payload_file.write_text(
        "".join(f"{item.marker_id}\t{item.payload}\n" for item in landmarks),
        encoding="ascii",
    )
    if args.png:
        render_pngs(landmarks, output)

    print(f"Validated {len(landmarks)} landmarks")
    print(f"Payloads: {payload_file}")
    if args.png:
        print(f"PNG files: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
