# RID branch integration

This branch keeps fixed-camera detection, SORT tracking, master selection, and
gimbal control unchanged. RID tracks are maintained independently by RID
identity. SORT and RID are combined only while constructing the current UI
packets; no cross-cycle identity binding is created or retained.

## Runtime configuration

RID is enabled when `ENABLE_RID=1` and `RID_PORT` is non-empty. The legacy
gimbal-camera ranging and laser are disabled by default on this branch.

The deployed Linux cabinet uses physical USB topology (`by-path`) rather than
the unstable `ttyUSBx` enumeration:

| Device | Linux default by-path |
| --- | --- |
| RID (USB HUB port 3) | `/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.1.3:1.0-port0` |
| GPS | `/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2:1.0-port0` |
| Gimbal | `/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.4:1.0-port0` |

Keep each device connected to its assigned physical USB socket. On Linux the
RID path above is the default, so `RID_PORT` is only needed as an override.
Windows still requires an explicit `RID_PORT=COMx` because its physical COM
assignment is installation-specific.

The systemd entry script waits for all three configured serial paths to exist
and be readable/writable before it starts Python. Missing hardware therefore
produces only a rate-limited `[HardwareWait]` journal message and does not
create timestamped run directories. Once all devices are available, Python
starts once and creates the normal run directory. The wait interval defaults
to 2 seconds and its unchanged-state journal interval defaults to 30 seconds;
they can be overridden with `HARDWARE_WAIT_INTERVAL` and
`HARDWARE_WAIT_LOG_INTERVAL`.

```powershell
$env:ENABLE_RID = "1"
$env:RID_PORT = "COM13"       # replace with the actual XP-MRID-04 port
$env:RID_BAUDRATE = "115200"
python main_tracking_v9.py
```

For an independent hardware check, stop the main process first and run:

```powershell
python tools_py/rid_serial_receiver_test.py --list-ports
python tools_py/rid_serial_receiver_test.py --port COM13 --show-raw
```

The standalone tool imports the production `RIDStreamParser`; it does not
contain a second parser implementation. Two processes cannot open the same
Windows COM port at the same time. Test captures are written under
`logs/rid_serial_test/` by default.

## Serial framing

The XP-MRID-04 link is opened as 115200 8N1. RS485/compatible serial and the
newer BLE stream use the same strict frame format:

```text
08 17 30 | LEN_L LEN_H | UTF-8 JSON | 3F 55
```

The little-endian length includes the two length bytes themselves:

```text
length_value = LEN_L | (LEN_H << 8)
json_length  = length_value - 2
frame_length = length_value + 5
```

The parser validates the header, length range, exact tail position, UTF-8,
JSON syntax, and top-level object type. It handles split frames, multiple
sticky frames in one read, and corruption resynchronization.

## Independent tracks and UI-time combination

SORT continues to maintain its own tracks from fixed-camera detections. RID
continues to maintain one track per RID identity and assigns stable UI IDs in
first-seen order. A RID broadcast may exist before any SORT target is detected;
it remains in the RID manager but no UI target packet is emitted until at least
one UI-eligible SORT track is available.

Only SORT tracks that have passed the existing UI confirmation and freshness
checks enter UI-time combination. On every UI send cycle, a rectangular
Hungarian assignment compares current true-north azimuth separation. This
assignment is used only to select a camera source and enforce the requested
count limit. It has no acceptance threshold, multi-frame confirmation,
ambiguity rejection, hold time, or stored RID-SORT binding.

The number of emitted targets is always:

```text
min(current UI-eligible SORT count, current valid RID count)
```

- If SORT has fewer tracks than RID, only the nearest-bearing one-to-one RID
  subset is sent.
- If counts are equal, every RID target is sent.
- If SORT has more tracks than RID, every RID target is sent.
- The assignment is recalculated from scratch on the next send cycle.

The existing UI binary packet layout is unchanged. For each emitted packet:

- target ID comes from the RID track's stable sequential `ui_id`;
- board/camera comes from the assigned SORT track's latest detection source;
- azimuth comes from RID station-to-UAV bearing;
- elevation comes from RID `AltGeo`, station GPS ellipsoid height, and
  horizontal distance;
- distance comes from RID station-to-UAV horizontal distance.

SORT angles and distances are not overwritten with RID values. SORT remains
the sole source used by the existing gimbal-control path.

## Coordinates and elevation

RID UAV coordinates and the station coordinates used for geometry are WGS-84.
The GPS thread retains a full-precision WGS-84 fix for RID calculations while
preserving the existing GCJ-02 UI GPS-position packet behavior. Do not directly
compare the RID WGS-84 coordinates with the UI's GCJ-02 station coordinates.

RID bearing is calculated from the station WGS-84 coordinate to the UAV WGS-84
coordinate. The result is a map azimuth with true north `0 deg`, east `90 deg`,
south `180 deg`, and west `270 deg`.

SORT keeps its existing device-relative azimuth internally. Only when current
azimuth separation is needed for UI-time assignment is it converted with:

```text
sort_map_az = (sort_relative_az + DEVICE_HEADING_DEG) % 360
```

RID does not provide a direct elevation-angle field. The current calculation is:

```text
station_ellipsoid_height = GGA altitude_msl + GGA geoid_separation
vertical_delta = UAVInfo.AltGeo - station_ellipsoid_height
rid_elevation = atan2(vertical_delta, horizontal_distance)
```

For the captured station frame, GGA MSL altitude `490.9668 m` plus geoid
separation `-42.8244 m` gives WGS-84 ellipsoid height `448.1424 m`, matching
the vendor application. If either station ellipsoid height or RID `AltGeo` is
unavailable, elevation is sent as `NaN` and the condition is logged. The RID
`Height` field is retained for diagnostics but is not used in this calculation.

`Lon=0` and `Lat=0` together are treated as an unavailable-position sentinel.
The protocol frame and RID identity remain valid, but `position_valid=False`;
no distance, bearing, elevation, or geographic trajectory point is generated.
A real point remains valid when only longitude or only latitude is zero.

## Logs

Each run directory contains:

- `raw_rid_serial_*.jsonl`: every serial `read()` chunk, exact hex bytes,
  decoded-frame metadata, parser-buffer size, and parser error counters.
- `raw_rid_*.jsonl`: every successfully decoded RID JSON object with local
  receive time.
- `rid_association_*.csv`: retained filename for compatibility; in the current
  design it records UI-time inputs, selection, and stateless pairs rather than
  persistent identity association.
- `events_*.csv`: UI sends, invalid RID positions, frame parse errors, missing
  station position, and count-limit skips.

Useful current events are:

- `RID_UI_FUSION_SUMMARY`: current SORT/RID counts and emitted pair count.
- `SORT_TRACK`: every UI-eligible SORT input, including relative/map azimuth
  and board/camera.
- `RID_TRACK`: every current valid RID input, including RID ID, sequential UI
  ID, bearing, elevation, distance, position, and whether it was selected.
- `RID_UI_STATELESS_PAIR`: the one-cycle camera/RID assignment and azimuth
  separation used to produce a packet.
- `RID_UI_COUNT_LIMIT_SKIP`: a RID target omitted because SORT count was lower.
- `RID_POSITION_INVALID`: a decoded RID identity carrying the `0/0` sentinel.
- `RID_UI_FUSION_SKIP`: station WGS-84 position is not yet available.
- `UI_STATUS_SEND`: final values and their sources; RID mode records
  `ui_az_source=rid_gps` and
  `ui_el_source=rid_altgeo_minus_station_altitude`.

To replay a serial capture, read `raw_rid_serial_*.jsonl` in order, convert
each `chunk_hex` using `bytes.fromhex(...)`, and feed the chunks unchanged to
`RIDStreamParser.feed()`.
