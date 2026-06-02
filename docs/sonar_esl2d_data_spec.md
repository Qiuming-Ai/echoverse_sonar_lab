# ESL2D Binary File Format Specification (v1)

## Scope

This document defines the on-disk binary format of `.esl2d` files produced by EchoVerse Sonar Lab for **2D imaging sonar** data.

An `.esl2d` file stores **beam-organized intensity profiles** (range bins per beam). It supports:

| `sonar_type` | Value | Typical beams | Bearing layout |
|---|---:|---|---|
| FLS | `0` | e.g. 256 | Uniform fan from `-beam_width/2` to `+beam_width/2` (e.g. ±65° when beam width is 130°) |
| SSS | `1` | 2 | Starboard `0°`, Port `180°` |

Each packet is one **frame** (one ping cycle). Data are stored **per beam**: for each beam, intensity samples cover detection range `0 … max_range_m`.

MBES may reuse the same container in future versions (`sonar_type = 2` reserved).

## Endianness

- All fixed-header numeric fields are **little-endian**.
- `beam_angles_deg` and `intensity` arrays are little-endian IEEE-754 `float32`.

## File Structure

Append-only sequence of packets (same philosophy as `.esl3d`):

1. Fixed header (`64` bytes)
2. Metadata JSON (`metadata_bytes`)
3. Beam bearings (`beam_angles_bytes`)
4. Intensity cube flattened beam-major (`intensity_bytes`)

No global file header.

## Packet Header (64 bytes)

| Offset | Size | Type | Field | Meaning |
|---:|---:|---|---|---|
| 0 | 4 | `u32` | `magic` | Constant `0x5032534E` (`"NS2P"`) |
| 4 | 2 | `u16` | `version` | Current version is `1` |
| 6 | 2 | `u16` | `header_bytes` | Must be `64` |
| 8 | 8 | `u64` | `seq` | Monotonic sequence number |
| 16 | 8 | `u64` | `timestamp_us` | Timestamp in microseconds |
| 24 | 2 | `u16` | `sonar_type` | `0=FLS`, `1=SSS` |
| 26 | 2 | `u16` | `reserved0` | Currently `0` |
| 28 | 4 | `u32` | `beam_count` | Number of beams in this frame |
| 32 | 4 | `u32` | `bin_count` | Range bins per beam |
| 36 | 4 | `float32` | `max_range_m` | Maximum detection range (m) |
| 40 | 4 | `u32` | `metadata_bytes` | Metadata payload size |
| 44 | 4 | `u32` | `beam_angles_bytes` | Bearing array size |
| 48 | 4 | `u32` | `intensity_bytes` | Intensity payload size |
| 52 | 4 | `u32` | `payload_bytes` | Sum of the three payload sections |
| 56 | 4 | `u32` | `reserved1` | Currently `0` |
| 60 | 4 | `u32` | `reserved2` | Currently `0` |

## Payload Layout

Immediately after the 64-byte header:

1. `metadata_bytes` bytes of UTF-8 JSON (compact)
2. `beam_angles_bytes` bytes of `float32` bearings in **degrees**
3. `intensity_bytes` bytes of `float32` intensity samples

Slicing:

- `metadata = payload[0 : metadata_bytes]`
- `beam_angles = payload[metadata_bytes : metadata_bytes + beam_angles_bytes]`
- `intensity = payload[metadata_bytes + beam_angles_bytes : payload_bytes]`

Expected sizes:

- `beam_angles_bytes == 4 * beam_count`
- `intensity_bytes == 4 * beam_count * bin_count`
- `payload_bytes == metadata_bytes + beam_angles_bytes + intensity_bytes`

## Intensity Array Layout

Beam-major, bin-minor (matches `sonar_types_v2::samples::Sonar::bins` storage):

```
index = beam * bin_count + bin
intensity[index]  // bin=0 nearest, bin=bin_count-1 farthest (approx. 0..max_range_m)
```

Approximate range of bin `k`:

```
range_k ≈ (k + 0.5) / bin_count * max_range_m
```

Exact two-way-time mapping is available in metadata (`bin_duration_s`, `speed_of_sound_mps`).

## Sonar-Type Semantics

### FLS (`sonar_type = 0`)

Example: 256 beams, ±65° coverage, 750 bins, `max_range_m = 30`.

- `beam_count = 256`
- `beam_angles_deg[i]` spans approximately `[-65°, +65°]` uniformly
- Each beam stores `bin_count` intensity samples along range `0 … max_range_m`

### SSS (`sonar_type = 1`)

Example: 2 beams (starboard + port), 512 bins, `max_range_m = 30`.

- `beam_count = 2`
- Beam order (fixed):
  - index `0`: **starboard**, bearing **`0°`**
  - index `1`: **port**, bearing **`180°`**
- Each beam stores `bin_count` intensity samples along range `0 … max_range_m`

## Metadata JSON Contract

Current writer includes:

- `byte_order`: `"little_endian"`
- `layout`: `"beam_major"`
- `data_order`: `"metadata_then_beam_angles_then_intensity"`
- `sonar_kind`: `"fls"` or `"sss"`
- `sonar_module_name`: human-readable module label
- `frame` object:
  - `seq`, `timestamp_us`
  - `sonar_type`, `beam_count`, `bin_count`, `max_range_m`
- `beams` array (one object per beam):
  - `index`
  - `bearing_deg`
  - `bin_count`
  - `range_start_m` (usually `0`)
  - `range_end_m` (usually `max_range_m`)
  - `side` (`"starboard"` / `"port"` for SSS; omitted for FLS)
- `pose` object: `x,y,z`, `yaw_deg`, `pitch_deg`, `quat_w,x,y,z`
- `sonar_config` object (runtime snapshot)
- `environment` object (runtime snapshot)

## Validation Rules

1. `magic == 0x5032534E`
2. `version == 1`
3. `header_bytes == 64`
4. `sonar_type` in `{0, 1}` for v1
5. `beam_count >= 1`, `bin_count >= 1`, `max_range_m > 0`
6. Payload length fields are self-consistent
7. EOF aligns with packet boundaries

## Reference Binary Struct

```c
#pragma pack(push, 1)
struct Esl2dHeaderV1 {
    uint32_t magic;              // 0x5032534E
    uint16_t version;            // 1
    uint16_t header_bytes;       // 64
    uint64_t seq;
    uint64_t timestamp_us;
    uint16_t sonar_type;         // 0 FLS, 1 SSS
    uint16_t reserved0;
    uint32_t beam_count;
    uint32_t bin_count;
    float    max_range_m;
    uint32_t metadata_bytes;
    uint32_t beam_angles_bytes;
    uint32_t intensity_bytes;
    uint32_t payload_bytes;
    uint32_t reserved1;
    uint32_t reserved2;
}; // 64 bytes
#pragma pack(pop)
```

## Streaming Parse Procedure

1. Read 64-byte header.
2. Decode as little-endian.
3. Validate header.
4. Read `payload_bytes`.
5. Split metadata / beam_angles / intensity.
6. Parse metadata JSON.
7. Reshape intensity to `[beam_count, bin_count]`.
8. Repeat until clean EOF.

## Compatibility Notes

- Mirrors `.esl3d` append-only packet design for offline pipelines.
- Future versions bump `version` and may extend metadata; parsers should honor `header_bytes` and explicit length fields.
