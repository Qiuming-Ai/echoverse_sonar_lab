# Sonar Point Cloud TCP Protocol

## Connection

- Transport: TCP (server in `multibeam_gui`)
- Default bind host: `0.0.0.0`
- Default port: `30001`
- Client connects to configured host/port and continuously reads point cloud frames.

## Frame Layout

Each frame is serialized as:

1. Fixed binary header (56 bytes, little-endian)
2. Metadata JSON bytes (UTF-8, compact JSON)
3. Range image bytes (`float32`, row-major)
4. Intensity image bytes (`float32`, row-major)

Data order is always:

- `range` first
- `intensity` second

## Dimension Policy

- `width = floor(horizontal_fov_deg / horizontal_angle_resolution_deg)`
- `height = floor(vertical_fov_deg / vertical_angle_resolution_deg)`
- Guard: width/height are clamped to at least 1 in code
- `point_count = width * height`

## Invalid Value Convention

- Range invalid value: `-1.0f`
- Intensity invalid value: `-1.0f`

## Fixed Header (56 bytes, little-endian)

| Offset | Type      | Field             | Description |
| ------ | --------- | ----------------- | ----------- |
| 0      | `u32`     | `magic`           | Constant `0x5033534E` (`"NS3P"`) |
| 4      | `u16`     | `version`         | Protocol version, current `1` |
| 6      | `u16`     | `header_bytes`    | Header length, fixed `56` |
| 8      | `u64`     | `seq`             | Frame sequence number |
| 16     | `u64`     | `timestamp_us`    | Frame timestamp (microseconds) |
| 24     | `u32`     | `width`           | Polar image width |
| 28     | `u32`     | `height`          | Polar image height |
| 32     | `u32`     | `point_count`     | `width * height` |
| 36     | `u32`     | `metadata_bytes`  | JSON metadata byte length |
| 40     | `u32`     | `range_bytes`     | Range payload byte length |
| 44     | `u32`     | `intensity_bytes` | Intensity payload byte length |
| 48     | `u32`     | `payload_bytes`   | `metadata + range + intensity` total bytes |
| 52     | `u32`     | reserved          | currently unused (set to 0 if present in future extension) |

> Note: Current implementation writes 12 fixed fields up to `payload_bytes`; parser should trust `header_bytes` and field lengths to split the payload.

## Metadata JSON

Main fields:

- `byte_order`: `"little_endian"`
- `layout`: `"row_major"`
- `data_order`: `"range_then_intensity"`
- `range_invalid_value`, `intensity_invalid_value`
- `frame`:
  - `seq`
  - `timestamp_us`
  - `width`
  - `height`
  - `point_count`
  - `rounding_policy` (`"floor"`)
- `sonar_config` (sent every frame):
  - `enabled`
  - `range_m`
  - `frequency_khz`
  - `bandwidth_khz`
  - `horizontal_angle_resolution_deg`
  - `vertical_angle_resolution_deg`
  - `horizontal_fov_deg`
  - `vertical_fov_deg`
- `environment` (sent every frame):
  - `enable_attenuation`
  - `attenuation_frequency_khz`
  - `temperature_c`
  - `salinity_ppt`
  - `acidity_ph`
  - `depth_m`
  - `enable_reverb`
  - `enable_speckle`
  - `sound_speed_mps`
- `pose` (sent every frame):
  - `x`, `y`, `z`
  - `yaw_deg`, `pitch_deg`
  - `quat_w`, `quat_x`, `quat_y`, `quat_z`

## Python Client Example

```python
import json
import socket
import struct
import numpy as np

HOST = "127.0.0.1"
PORT = 30001

HEADER_FMT = "<IHHQQIIIIIIII"  # 56 bytes
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAGIC = 0x5033534E

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf.extend(chunk)
    return bytes(buf)

with socket.create_connection((HOST, PORT), timeout=5.0) as sock:
    while True:
        hdr = recv_exact(sock, HEADER_SIZE)
        (
            magic, version, header_bytes, seq, timestamp_us,
            width, height, point_count,
            metadata_bytes, range_bytes, intensity_bytes, payload_bytes, reserved
        ) = struct.unpack(HEADER_FMT, hdr)

        if magic != MAGIC:
            raise ValueError(f"bad magic: {hex(magic)}")
        if version != 1:
            raise ValueError(f"unsupported version: {version}")

        payload = recv_exact(sock, payload_bytes)
        meta_raw = payload[:metadata_bytes]
        range_raw = payload[metadata_bytes:metadata_bytes + range_bytes]
        intensity_raw = payload[metadata_bytes + range_bytes:metadata_bytes + range_bytes + intensity_bytes]

        metadata = json.loads(meta_raw.decode("utf-8"))

        range_img = np.frombuffer(range_raw, dtype="<f4").reshape((height, width))
        intensity_img = np.frombuffer(intensity_raw, dtype="<f4").reshape((height, width))

        print(
            f"seq={seq} ts={timestamp_us} size={width}x{height} "
            f"pose=({metadata['pose']['x']:.2f}, {metadata['pose']['y']:.2f}, {metadata['pose']['z']:.2f})"
        )
```

## C++ Parsing Sketch

```cpp
// 1) read fixed header
Header h;
readExact(sock, &h, sizeof(h));
validateMagicAndVersion(h);

// 2) read metadata json + range + intensity
std::vector<uint8_t> payload(h.payload_bytes);
readExact(sock, payload.data(), payload.size());

const uint8_t* p = payload.data();
std::string metadata_json(reinterpret_cast<const char*>(p), h.metadata_bytes);
p += h.metadata_bytes;

const float* range_data = reinterpret_cast<const float*>(p);
p += h.range_bytes;
const float* intensity_data = reinterpret_cast<const float*>(p);

// 3) row-major index
// idx = v * width + h
```
