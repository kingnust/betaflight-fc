# Experimental BW21 QR Localization

This system uses QR landmarks to anchor the drone's optical-flow dead reckoning
to known map coordinates. It is deliberately advisory: it reports a world
position estimate but does not change motor outputs, arming, Position Hold
targets, or Altitude Hold.

## Data path

1. The BW21 decodes a QR code with quirc or the timed ZBar fallback.
2. It measures the QR center, apparent side length, image area, and top-edge
   rotation from the detected corners.
3. The camera sends the text and geometry over the acknowledged, CRC-protected
   MSP2 `0x3001` WK2132 camera UART protocol. Protocol v2 carries geometry;
   the FC still accepts legacy text-only v1 messages for diagnostics.
4. The FC parses a strict landmark payload and estimates the camera position.
5. Accepted observations update a filtered map offset over the existing local
   optical-flow position. Movement between scans continues to come from flow.

The camera publishes a still-visible landmark at up to 4 Hz. This gives the
filter repeated measurements without tying the camera frame rate to UART
traffic.

## Landmark format

Encode this ASCII text in each QR code:

```text
DLOC1,<id>,<x_m>,<y_m>,<z_m>,<edge_yaw_deg>,<size_m>
```

Example:

```text
DLOC1,A1,1.000,2.000,0.000,90.0,0.200
```

- `id` is 1 to 15 letters, digits, `_`, or `-`.
- `x_m`, `y_m`, and `z_m` are the QR center in map coordinates, in meters.
- `edge_yaw_deg` is the world heading from the printed QR's top-left corner to
  its top-right corner. It is used only as a diagnostic heading estimate for a
  downward camera.
- `size_m` is the measured outside width of the printed QR square, not the
  paper width or quiet-zone width.
- The complete payload must fit in 96 ASCII bytes.

Map axes are right-handed: `+X` is map forward, `+Y` is map right when yaw is
zero, and `+Z` is up. FC yaw zero points the drone nose along `+X`.

## Generate markers

Edit `tools/qr_localization/landmarks.example.csv`, then run:

```powershell
python tools/qr_localization/generate_landmarks.py `
  tools/qr_localization/landmarks.example.csv
```

This always validates the map and writes `payloads.txt`. Add `--png` after
installing `qrcode[pil]` to create printable PNG files:

```powershell
python -m pip install "qrcode[pil]"
python tools/qr_localization/generate_landmarks.py `
  tools/qr_localization/landmarks.example.csv --png
```

Print without page scaling and verify the finished square with a ruler. A size
error directly becomes a range error.

## Firmware and wiring

Use these matching builds:

```powershell
# Camera repository
platformio run -e bw21-cam-vision-wk2132

# Flight-controller repository
platformio run -e drone_proto_esp32s3_wk2132_experimental
```

Cross the 3.3 V UART signals and join grounds:

| BW21 | WK2132 camera port |
| --- | --- |
| D21 / UART0_OUT | CAMERA_RX |
| D22 / UART0_IN | CAMERA_TX |
| GND | GND |

Power the BW21 from regulated 5 V, not from the FC 3.3 V rail.

## Bench workflow

Remove propellers. Start with the camera and FC stationary.

1. Open the BW21 web page, select `Vision`, and scan an ordinary QR code. This
   proves decoding without involving localization.
2. Scan a `DLOC1` marker and inspect `/api/vision`. Its `qr.geometry.valid`
   field must be true, center values should move with the marker, and apparent
   side should increase as the camera approaches it.
3. In the FC CLI, run `camstatus`. Confirm protocol `2`, increasing accepted
   count, the complete QR text, and nonzero geometry.
4. Select the physical mount and clear prior state:

```text
qrloc mount down
qrloc clear
```

Use `qrloc mount front` for a forward-facing camera and wall markers.

5. Scan one marker repeatedly and run `qrloc`. Check `valid=1`, `fresh=1`, the
   landmark ID, confidence, observed position, filtered world position, range,
   and rejection counters.
6. Move the unpowered assembly a measured distance over a textured surface.
   The world estimate should move continuously with optical flow, then correct
   gradually when another marker is seen.

`qrloc enable 0` stops accepting localization observations without disabling
camera QR telemetry. `qrloc enable 1` re-enables it. `qrloc clear` resets the
map offset and counters.

## Observation gates

The FC rejects observations when:

- the text is not a `DLOC1` landmark or its fields/ranges are malformed;
- the decoder did not provide geometry;
- the QR occupies less than 2.5% of image width;
- roll or pitch exceeds 25 degrees;
- QR size gives an impossible range;
- downward QR range and MTF02P range disagree by more than 55%;
- a correction jumps more than 1.5 m horizontally or 1.0 m vertically.

A large correction must appear twice from the same landmark within one second
and agree within 0.35 m. Even then, only a limited fraction is blended. Normal
correction gain varies with QR size, centering, and range agreement. Confidence
decays to zero 2.5 seconds after the last accepted observation.

## Calibration

The default field of view is 72 degrees horizontal and 55 degrees vertical.
These are starting values, not a final camera calibration. Measure calibration
at several known distances and adjust `horizontalFovDeg` and `verticalFovDeg`
in `QrLocalizationConfig` if range or off-center position has a consistent
bias. Keep the camera rigid, measure its orientation, and keep the printed QR
flat. Lens distortion, motion blur, oblique views, vibration, and inaccurate
marker size all reduce accuracy.

For flight use, first log stationary and hand-moved bench data, then tethered
low-altitude data. Do not connect this estimate to Position Hold targets until
its rejection behavior and errors have been measured on the final camera mount.
