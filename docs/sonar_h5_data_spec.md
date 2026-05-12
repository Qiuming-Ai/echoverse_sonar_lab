# Sonar H5 Data Specification (Based on `DataMakerInit.m`)

## 1. Purpose

This document describes how sonar `.h5` output is initialized in the current pipeline, based on `src/matlab_point2file2image/Initialize/DataMakerInit.m`.

`DataMakerInit` does not directly write signal matrices itself; it builds `sonarInfo`, resolves output path, and calls:

- `dataMaker = SonarDataMaker();`
- `dataMaker.start(sonar_h5_filename, sonarInfo);`

So this spec focuses on:

1. Required input fields in `sonar`
2. Path and naming rules for `.h5`
3. Metadata contract (`sonarInfo`) passed to `SonarDataMaker`

## 2. Required Input Fields

`DataMakerInit` hard-fails if any of the following fields is missing:

- `sonar.tx_type`
- `sonar.decimation_factor`
- `sonar.MF`
- `sonar.esl3d_path` (must be non-empty)

Validation behavior:

- Missing field => throws `DataMakerInit:MissingVar` with guidance to run `SonarInit` first.

## 3. Output H5 File Naming and Directory

The output filename is derived from `esl3d_path`:

1. Extract `esl3d_dir` and `esl3d_name` from `sonar.esl3d_path`
2. Resolve output directory:
   - if `sonar.output_path` exists and is non-empty: use it
   - otherwise: use `esl3d_dir`
3. Final path:
   - `<output_dir>/<esl3d_name>.h5`

Example:

- input `esl3d_path = D:/proj/Point Cloud/run_01.esl3d`
- no `output_path`
- output H5 = `D:/proj/Point Cloud/run_01.h5`

## 4. Transmission Mode Mapping

`sonar.tx_type` controls array type metadata:

- `cdm` -> `array_type = "CDM"`
- `fdm` -> `array_type = "FDM"`
- otherwise -> `array_type = "Baseline"`

Internal naming prefix (`CDMData` / `FDMData` / `BaselineData`) is selected in code but currently not used for final filename in this function.

## 5. Receive Window Policy

If `sonar.array_window` exists:

- string/scalar string values:
  - `"hamming"` -> `hamming(sonar.Nrx)`
  - `"hann"` -> `hann(sonar.Nrx)`
  - `"blackman"` -> `blackman(sonar.Nrx)`
  - otherwise -> `ones(sonar.Nrx, 1)`
- non-string value:
  - used directly as custom window
  - `signal_win = "custom"`

If `sonar.array_window` is absent:

- default window is `hamming(sonar.Nrx)`
- `signal_win = "hamming"`

## 6. Sector and Scan-Angle Policy

- `sector_div`:
  - only created when `sonar.angles_div` is a non-empty cell array
  - generated from first element start + each sector end
- `sector_num`:
  - uses `sonar.sector_num` if present
  - default = `1`
- `scan_angle`:
  - uses `sonar.scan_angles` if present
  - default = `-60:1:60`

## 7. Frequency/Bandwidth Selection Rule

Center frequency:

- use `sonar.Subfc` if present and `numel(Subfc) > 1`
- else use `sonar.fc` if present
- else empty

Bandwidth:

- use `sonar.SubBW` if present and `numel(SubBW) > 1`
- else use `sonar.BW` if present
- else empty

## 8. `sonarInfo` Metadata Contract

The following fields are populated and passed into `SonarDataMaker.start(...)`:

- `array_type`
- `signal_type` = `"Baseband"`
- `signal_win`
- `bandwidth`
- `sampling_frequency` <- `sonar.fs`
- `center_frequency`
- `decimate_factor` <- `sonar.decimation_factor`
- `sector_num`
- `match_filter_data` <- `sonar.MF_deci`
- `receive_array_num` <- `sonar.Nrx`
- `receive_array_position` <- `sonar.rx_xyz`
- `receive_array_win`
- `pulse_duration` <- `sonar.pulse_len`
- `sound_velocity` <- `sonar.c0`
- `velocity` <- `sonar.velocity`
- `snr_level` <- `sonar.snr_level`
- `timestamp` <- runtime string `yyyymmdd_HHMMSS`
- `scan_angle`
- optional `sector_div` (when available)
- optional `sample_delay` <- `sonar.compensate_range` (when available)

## 9. Relationship to ESL3D

`DataMakerInit` uses `.esl3d` primarily as:

1. an identity anchor for output naming (`<esl3d_name>.h5`)
2. a default output directory source (`<esl3d_dir>`)

It does not parse `.esl3d` content in this function.

## 10. Practical Integration Notes

- Ensure `SonarInit` is executed before `DataMakerInit`, otherwise required fields may be missing.
- If your workflow requires deterministic output location, always set `sonar.output_path`.
- If downstream processing depends on sector boundaries, provide valid `angles_div`.
- If you use custom receive windows, verify size equals `Nrx`.
