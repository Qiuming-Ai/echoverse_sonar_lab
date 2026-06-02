# Sonar TCP Protocol

## Scope

EchoVerse Sonar Lab streams sonar data over TCP using a **packet-per-frame** binary layout. Two frame families share the same transport model (fixed header + length-prefixed payload) but differ in magic and payload semantics:

| Magic | ASCII | Format | Typical use | Default port* |
|------:|-------|--------|-------------|---------------|
| `0x5033534E` | `NS3P` | ESL3D | 3D point cloud (polar range + intensity images) | `30001` |
| `0x5032534E` | `NS2P` | ESL2D | 2D imaging sonar (beam-organized intensity profiles) | FLS/MBES `30001`, SSS `30002` |

\* Ports are configurable per module in the sonar settings panel. A `.esl3d` or `.esl2d` file is an append-only concatenation of the same packets sent on TCP.

All multi-byte integers and `float32` arrays are **little-endian**.

## Connection

- Transport: TCP server inside EchoVerse Sonar Lab
- Default bind host: `0.0.0.0`
- Client connects to the configured host/port and reads frames continuously until disconnect
- Server accepts **one client** at a time; a new connection replaces the previous client

## Frame Dispatch

1. Read the first 4 bytes as `u32 magic` (little-endian).
2. Branch:
   - `magic == 0x5033534E` → ESL3D frame (56-byte header)
   - `magic == 0x5032534E` → ESL2D frame (64-byte header)
3. Read the remaining `header_bytes - 4` bytes to complete the fixed header.
4. Read `payload_bytes` and split according to the format below.

---

## ESL3D — 3D Point Cloud (`NS3P`)

Produced by FLS/MBES point-cloud output. Each frame is one polar range/intensity image pair plus metadata.

### Frame Layout

1. Fixed binary header (56 bytes)
2. Metadata JSON (UTF-8, compact)
3. Range image (`float32`, row-major)
4. Intensity image (`float32`, row-major)

Payload order: `metadata` → `range` → `intensity`.

### Dimension Policy

- `width = floor(horizontal_fov_deg / horizontal_angle_resolution_deg)`
- `height = floor(vertical_fov_deg / vertical_angle_resolution_deg)`
- Guard: width/height clamped to at least 1
- `point_count = width * height`
- `range_bytes = intensity_bytes = 4 * width * height`

### Invalid Value Convention

- Range invalid value: `-1.0f`
- Intensity invalid value: `-1.0f`

### Fixed Header (56 bytes)

| Offset | Type  | Field             | Description |
|-------:|-------|-------------------|-------------|
| 0      | `u32` | `magic`           | `0x5033534E` (`"NS3P"`) |
| 4      | `u16` | `version`         | `1` |
| 6      | `u16` | `header_bytes`    | `56` |
| 8      | `u64` | `seq`             | Frame sequence number |
| 16     | `u64` | `timestamp_us`    | Timestamp (µs) |
| 24     | `u32` | `width`           | Polar image width |
| 28     | `u32` | `height`          | Polar image height |
| 32     | `u32` | `point_count`     | `width * height` |
| 36     | `u32` | `metadata_bytes`  | JSON length |
| 40     | `u32` | `range_bytes`     | Range payload length |
| 44     | `u32` | `intensity_bytes` | Intensity payload length |
| 48     | `u32` | `payload_bytes`   | `metadata + range + intensity` |
| 52     | `u32` | reserved          | `0` |

Validate: `payload_bytes == metadata_bytes + range_bytes + intensity_bytes`.

### Metadata JSON (ESL3D)

- `byte_order`: `"little_endian"`
- `layout`: `"row_major"`
- `data_order`: `"range_then_intensity"`
- `range_invalid_value`, `intensity_invalid_value`
- `frame`: `seq`, `timestamp_us`, `width`, `height`, `point_count`, `rounding_policy`
- `sonar_config`: range, frequency, bandwidth, FOV, angle resolutions, …
- `environment`: attenuation, temperature, salinity, sound speed, …
- `pose`: `x,y,z`, `yaw_deg`, `pitch_deg`, `quat_w,x,y,z`

Row-major index: `idx = row * width + col`; shape `(height, width)`.

---

## ESL2D — 2D Imaging Sonar (`NS2P`)

Produced by FLS / MBES / SSS when **Enable TCP Output** is on in the 2D sonar settings panel. Each frame is one ping stored **per beam**.

| `sonar_type` | Value | Typical source | Typical beams | Bearing layout |
|---:|---:|---|---|---|
| FLS-compatible | `0` | FLS and current MBES ESL2D writer | e.g. 256 | Uniform fan `[-beam_width/2, +beam_width/2]` (e.g. ±65°) |
| SSS | `1` | SSS | 2 | Starboard `0°`, Port `180°` |

### Frame Layout

1. Fixed binary header (64 bytes)
2. Metadata JSON (UTF-8, compact)
3. Beam bearings (`float32[beam_count]`, degrees)
4. Intensity (`float32[beam_count * bin_count]`, beam-major)

Payload order: `metadata` → `beam_angles` → `intensity`.

### Fixed Header (64 bytes)

| Offset | Type     | Field               | Description |
|-------:|----------|---------------------|-------------|
| 0      | `u32`    | `magic`             | `0x5032534E` (`"NS2P"`) |
| 4      | `u16`    | `version`           | `1` |
| 6      | `u16`    | `header_bytes`      | `64` |
| 8      | `u64`    | `seq`               | Frame sequence number |
| 16     | `u64`    | `timestamp_us`      | Timestamp (µs) |
| 24     | `u16`    | `sonar_type`        | `0=FLS`, `1=SSS` |
| 26     | `u16`    | `reserved0`         | `0` |
| 28     | `u32`    | `beam_count`        | Beams in this frame |
| 32     | `u32`    | `bin_count`         | Range bins per beam |
| 36     | `float32`| `max_range_m`       | Detection range 0…max (m) |
| 40     | `u32`    | `metadata_bytes`    | JSON length |
| 44     | `u32`    | `beam_angles_bytes` | `4 * beam_count` |
| 48     | `u32`    | `intensity_bytes`   | `4 * beam_count * bin_count` |
| 52     | `u32`    | `payload_bytes`     | Sum of three payload sections |
| 56     | `u32`    | `reserved1`         | `0` |
| 60     | `u32`    | `reserved2`         | `0` |

Validate:

- `beam_angles_bytes == 4 * beam_count`
- `intensity_bytes == 4 * beam_count * bin_count`
- `payload_bytes == metadata_bytes + beam_angles_bytes + intensity_bytes`

### Intensity Layout

Beam-major, bin-minor:

```
index = beam * bin_count + bin
range_k ≈ (bin + 0.5) / bin_count * max_range_m
```

### SSS Beam Order

- Index `0`: **starboard**, bearing **0°**
- Index `1`: **port**, bearing **180°**

### Metadata JSON (ESL2D)

- `byte_order`: `"little_endian"`
- `layout`: `"beam_major"`
- `data_order`: `"metadata_then_beam_angles_then_intensity"`
- `sonar_kind`: `"fls"` or `"sss"` (`"fls"` is also used by current MBES ESL2D output)
- `sonar_module_name`
- `frame`: `seq`, `timestamp_us`, `sonar_type`, `beam_count`, `bin_count`, `max_range_m`
- `beams[]`: per-beam `index`, `bearing_deg`, `bin_count`, `range_start_m`, `range_end_m`, optional `side`
- `pose`, `sonar_config`, `environment`

Reshape intensity to `[beam_count, bin_count]`.

---

## Python Client — ESL3D

```python
import json
import socket
import struct
import numpy as np

HOST = "127.0.0.1"
PORT = 30001

ESL3D_MAGIC = 0x5033534E
ESL3D_HEADER_FMT = "<IHHQQIIIIIIII"
ESL3D_HEADER_SIZE = struct.calcsize(ESL3D_HEADER_FMT)

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf.extend(chunk)
    return bytes(buf)

def read_esl3d_frame(sock: socket.socket):
    hdr = recv_exact(sock, ESL3D_HEADER_SIZE)
    (
        magic, version, header_bytes, seq, timestamp_us,
        width, height, point_count,
        metadata_bytes, range_bytes, intensity_bytes, payload_bytes, reserved
    ) = struct.unpack(ESL3D_HEADER_FMT, hdr)
    if magic != ESL3D_MAGIC:
        raise ValueError(f"expected ESL3D magic, got {hex(magic)}")
    if version != 1:
        raise ValueError(f"unsupported ESL3D version: {version}")

    payload = recv_exact(sock, payload_bytes)
    meta_raw = payload[:metadata_bytes]
    range_raw = payload[metadata_bytes:metadata_bytes + range_bytes]
    intensity_raw = payload[metadata_bytes + range_bytes:metadata_bytes + range_bytes + intensity_bytes]
    metadata = json.loads(meta_raw.decode("utf-8"))
    range_img = np.frombuffer(range_raw, dtype="<f4").reshape((height, width))
    intensity_img = np.frombuffer(intensity_raw, dtype="<f4").reshape((height, width))
    return seq, timestamp_us, metadata, range_img, intensity_img

with socket.create_connection((HOST, PORT), timeout=5.0) as sock:
    while True:
        seq, ts, meta, r, i = read_esl3d_frame(sock)
        pose = meta["pose"]
        print(f"[ESL3D] seq={seq} ts={ts} size={r.shape} pose=({pose['x']:.2f},{pose['y']:.2f},{pose['z']:.2f})")
```

## Python Client — ESL2D

```python
import json
import socket
import struct
import numpy as np

HOST = "127.0.0.1"
PORT = 30001  # SSS default is often 30002

ESL2D_MAGIC = 0x5032534E
ESL2D_HEADER_FMT = "<IHHQQHHIIfIIIIII"
ESL2D_HEADER_SIZE = struct.calcsize(ESL2D_HEADER_FMT)

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf.extend(chunk)
    return bytes(buf)

def read_esl2d_frame(sock: socket.socket):
    hdr = recv_exact(sock, ESL2D_HEADER_SIZE)
    (
        magic, version, header_bytes, seq, timestamp_us,
        sonar_type, reserved0, beam_count, bin_count, max_range_m,
        metadata_bytes, beam_angles_bytes, intensity_bytes, payload_bytes,
        reserved1, reserved2
    ) = struct.unpack(ESL2D_HEADER_FMT, hdr)
    if magic != ESL2D_MAGIC:
        raise ValueError(f"expected ESL2D magic, got {hex(magic)}")
    if version != 1:
        raise ValueError(f"unsupported ESL2D version: {version}")

    payload = recv_exact(sock, payload_bytes)
    p0 = 0
    p1 = metadata_bytes
    p2 = p1 + beam_angles_bytes
    metadata = json.loads(payload[p0:p1].decode("utf-8"))
    beam_angles = np.frombuffer(payload[p1:p2], dtype="<f4")
    intensity = np.frombuffer(payload[p2:p2 + intensity_bytes], dtype="<f4").reshape((beam_count, bin_count))
    return seq, timestamp_us, sonar_type, max_range_m, metadata, beam_angles, intensity

with socket.create_connection((HOST, PORT), timeout=5.0) as sock:
    while True:
        seq, ts, sonar_type, max_range_m, meta, angles, intensity = read_esl2d_frame(sock)
        kind = meta.get("sonar_kind", "fls" if sonar_type == 0 else "sss")
        print(f"[ESL2D] seq={seq} kind={kind} beams={intensity.shape[0]} bins={intensity.shape[1]} range={max_range_m:.1f}m")
```

## Python Client — Auto-dispatch

```python
import struct

def read_next_frame(sock):
    magic_raw = recv_exact(sock, 4)
    (magic,) = struct.unpack("<I", magic_raw)
    if magic == 0x5033534E:
        rest = recv_exact(sock, ESL3D_HEADER_SIZE - 4)
        # re-parse full ESL3D header + payload ...
    elif magic == 0x5032534E:
        rest = recv_exact(sock, ESL2D_HEADER_SIZE - 4)
        # re-parse full ESL2D header + payload ...
    else:
        raise ValueError(f"unknown magic {hex(magic)}")
```

Peeking the first `u32` before choosing header size is the recommended multiplexing strategy when the port may carry either format.

## C++ Parsing Sketch — ESL3D

```cpp
struct Esl3dHeaderV1 {
    uint32_t magic;           // 0x5033534E
    uint16_t version;         // 1
    uint16_t header_bytes;    // 56
    uint64_t seq;
    uint64_t timestamp_us;
    uint32_t width, height, point_count;
    uint32_t metadata_bytes, range_bytes, intensity_bytes, payload_bytes;
    uint32_t reserved;
};

// read header → read payload_bytes → split metadata / range / intensity
// row-major: idx = row * width + col
```

## C++ Parsing Sketch — ESL2D

```cpp
struct Esl2dHeaderV1 {
    uint32_t magic;           // 0x5032534E
    uint16_t version;         // 1
    uint16_t header_bytes;    // 64
    uint64_t seq;
    uint64_t timestamp_us;
    uint16_t sonar_type;      // 0 FLS, 1 SSS
    uint16_t reserved0;
    uint32_t beam_count, bin_count;
    float max_range_m;
    uint32_t metadata_bytes, beam_angles_bytes, intensity_bytes, payload_bytes;
    uint32_t reserved1, reserved2;
};

// read header → read payload_bytes
// metadata = payload[0 : metadata_bytes]
// beam_angles = float32[beam_count]
// intensity = float32[beam_count * bin_count], beam-major
```

## Related Specifications

- ESL3D on-disk details: `docs/sonar_esl3d_data_spec.md`
- ESL2D on-disk details: `docs/sonar_esl2d_data_spec.md`
- MATLAB readers: `src/matlab_point2file2image/ESL3D/esl3d.m`, `src/matlab_point2file2image/ESL2D/esl2d.m`

## Compatibility Notes

- Both formats use `version = 1` today. Future versions must bump `version` and may extend headers; parsers should honor `header_bytes` and explicit length fields.
- TCP packets and file records are byte-identical for a given frame.
