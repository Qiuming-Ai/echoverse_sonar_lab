# Model Loading Strategy and Custom Asset Integration

This document explains how EchoVerse Sonar Lab currently loads scene models, which file formats are supported in practice, and how to add your own models and materials into the software.

## 1. Runtime Model Loading Strategy

The runtime scene pipeline is implemented primarily in `src/SharedScene.cpp`.

### 1.1 World Selection and Resolution

At startup, the app resolves the scene world using this order:

1. If `scene.world` is an existing absolute/relative file path, use it directly.
2. Otherwise, try resolving it relative to the repository/runtime root.
3. Otherwise, treat it as a scene key and use:
   - `uwmodels/scenes/<world_key>/<world_key>.world`

If the world file is unreadable:

- The engine falls back to a built-in static scene map (`ssiv_bahia` or `tank`).

If the world file is readable but has zero `<include>` entries:

- The preview scene is intentionally empty.

### 1.2 Include Parsing (`.world`)

The world parser reads every `<include>` block and extracts:

- `<uri>` (usually `model://<model_name>`)
- `<pose>` (`x y z roll pitch yaw`)

Each include becomes one model instance candidate.

### 1.3 Model Geometry Parsing (`model.sdf`)

For each include, the loader reads:

- `uwmodels/sdf/<model_name>/model.sdf`

Then it builds renderable geometry from:

- `<visual>` blocks (preferred visible geometry)
- `<collision>` blocks (loaded too, hidden when visual geometry exists)

For each geometry block, the loader tries in this order:

1. Mesh from `<mesh><uri>...</uri></mesh>`
2. Primitive fallback from SDF geometry:
   - `<box><size>...</size></box>`
   - `<cylinder><radius>...</radius><length>...</length></cylinder>`
   - `<sphere><radius>...</radius></sphere>`
   - `<plane><size>...</size></plane>`
3. Generic fallback shape if none is available

Final pose is computed by adding:

- world include pose
- model-level `<pose>`
- geometry-level `<pose>`

### 1.4 Mesh Loading and Fallback Logic

When loading a mesh path, the runtime uses:

1. Built-in STL loader first for `.stl` files (binary and ASCII)
2. Otherwise `osgDB::readRefNodeFile(...)` (OpenSceneGraph plugin-based loader)
3. If loading `.osgb` fails, try `collision.stl` in the same model directory
4. If all loading attempts fail, create a fallback primitive node

This design keeps scenes usable even when some plugins/assets are missing.

### 1.5 Material Resolution and Override Priority

Material parsing supports both SDF values and script-based materials.

For each geometry block:

1. Parse `<material><script>`:
   - `<uri>`: material script file location
   - `<name>`: material entry name
2. Parse script keywords (if found):
   - `ambient`, `diffuse`, `specular`, `emissive`
   - `texture_unit` -> `texture`, `filtering`, `max_anisotropy`, `scale`
3. Merge SDF material fields (`<ambient>`, `<diffuse>`, `<emissive>`) for missing values
4. If still missing but a material block exists, infer fallback color from material name
5. Apply material recursively to loaded OSG nodes

Important behavior:

- If no material override is parsed at all, the loader keeps the mesh's original embedded material/shading.

## 2. Supported File Types

Support is split between explicit built-in handling and OSG plugin-dependent handling.

### 2.1 Explicitly Handled by This Codebase

- World files: `.world` (XML include parsing)
- Model definition: `model.sdf` (SDF-style XML parsing)
- Mesh: `.stl` (binary + ASCII, built-in parser)
- Mesh fallback path for failed `.osgb` loads: `collision.stl`
- Typical material scripts: text-based script files referenced from SDF (commonly `.material`)

### 2.2 Loaded Through OpenSceneGraph Plugins

Any mesh/image format supported by installed OSG plugins can be used via `osgDB`.

In practice, common plugin-backed formats include (depending on your runtime package):

- Mesh/scene: `.osgb`, `.dae` (COLLADA), and other plugin-enabled formats
- Texture/image: `.png`, `.jpg/.jpeg`, `.bmp`, `.tga`, `.gif`, and others supported by available image plugins

Notes:

- Real support depends on whether plugin DLLs are present next to the executable (or in configured plugin search paths).
- The CMake setup is designed to deploy OSG plugin folders and COLLADA runtime dependencies for packaged Windows builds.

## 3. How to Add Your Own Model

You can add models in two ways:

- Through the Scene Editor panel (recommended for existing `uwmodels/sdf` models)
- By manually creating/editing world and model files

### 3.1 Required Directory Structure

Place your model under:

- `uwmodels/sdf/<your_model_name>/`

Minimum required files:

- `uwmodels/sdf/<your_model_name>/model.sdf`
- Mesh files referenced by `model.sdf` (for example `visual.dae`, `visual.osgb`, `collision.stl`)

### 3.2 Add Model via Scene Editor UI

The Scene Editor (`Add Model...`) scans model names from directories that contain:

- `uwmodels/sdf/<name>/model.sdf`

Workflow:

1. Ensure your `model.sdf` exists in the correct folder.
2. Open the project in `echoverse_sonar_lab`.
3. Enable **Scene Edit Mode**.
4. Click **Add Model...** and choose your model.
5. Set pose values.
6. Save/reload preview (the panel writes `<include>` entries to the project world file).

### 3.3 Add Model by Editing `.world` Directly

Add an include block to your project world file:

```xml
<include>
  <uri>model://my_custom_model</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

Then reload the scene (or restart the app).

## 4. How to Add Your Own Material/Texture

### 4.1 In `model.sdf` Geometry Block

Inside a `<visual>` block, define material via script and/or direct color values:

```xml
<material>
  <script>
    <uri>model://my_custom_model/materials/scripts/my.material</uri>
    <name>My/Material</name>
  </script>
  <diffuse>0.7 0.7 0.7 1.0</diffuse>
  <ambient>0.4 0.4 0.4 1.0</ambient>
  <emissive>0.0 0.0 0.0 1.0</emissive>
</material>
```

### 4.2 Material Script Keys Read by the Loader

In the material script entry, the parser currently reads:

- `ambient`
- `diffuse`
- `specular`
- `emissive`
- `texture_unit` section:
  - `texture <path>`
  - `filtering <mode>`
  - `max_anisotropy <value>`
  - `scale <sx> <sy>`

### 4.3 Texture Path Rules

URI/path conversion supports:

- `model://...` -> resolved into `uwmodels/sdf/...`
- `file://Media/...` -> resolved into `uwmodels/Media/...`
- `file://media/...` -> resolved into `uwmodels/media/...`
- Other relative paths are resolved relative to the material file directory

## 5. Practical Compatibility Recommendations

For highest success across machines/builds:

- Always provide `collision.stl` (acts as a robust fallback)
- Keep one valid visual mesh and one collision mesh per model
- Prefer relative paths inside `uwmodels` instead of absolute disk paths
- Package OSG plugins together with the executable
- If your visual mesh fails to load, verify plugin availability first

## 6. Quick Troubleshooting

- Model not listed in **Add Model...**
  - Check `uwmodels/sdf/<name>/model.sdf` exists.
- Model appears as a box/primitive
  - Mesh URI path cannot be resolved, or required plugin is missing.
- `.osgb` model not visible
  - Loader may fall back to `collision.stl`; verify plugin deployment and mesh file.
- Texture not visible
  - Check material script texture path and image plugin availability.
- Material colors not applied
  - Confirm material block is under the same geometry block that is actually loaded.

---

Implementation references:

- `src/SharedScene.cpp`
- `src/WorldFileIo.cpp`
- `src/SceneEditorPanel.cpp`
- `CMakeLists.txt`
