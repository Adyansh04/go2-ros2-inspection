# go2_worlds

Gazebo Harmonic worlds and procedural world/texture generators for the Go2 inspection and exploration simulation stack.

## Overview

`go2_worlds` is an `ament_cmake` package that ships the SDF worlds the Go2 simulation runs in and the Python scripts that build them. The worlds range from a fast 4-room maze for exploration testing up to a richly-populated multi-room inspection facility with analog gauges, props, fire, and a walking human actor. Worlds are launched indirectly through `go2_bringup` launch files via a `world:=` argument; this package itself contains no ROS nodes or launch files.

The generator scripts (`scripts/gen_*.py`) procedurally synthesize the SDF worlds, gauge-face textures, fire particle sprites, and a rasterized occupancy map. Several worlds are *generated and additive* (a base shell plus injected gauges/props), and several embed absolute texture paths via a per-machine no-space symlink, so the generators are intended to be re-run on each machine.

## Worlds

Installed to `share/go2_worlds/worlds/` and selected with `world:=<file>` in the bringup launch files.

| World file | Generator | Description |
|---|---|---|
| `maze.sdf` | `gen_maze_world.py` | Compact 12x8 m box split into 4 rooms (NW/NE/SW/SE) around a central hub, doorway gaps connecting all four. One analog gauge per room (one seeded anomaly). Robot spawns at (0,0) facing +X. Fast to map; exercises the full explore->map->zones->inspect->read pipeline. |
| `lab.sdf` | hand-authored | Stage-1 test world: a simple enclosed room with obstacles to map. Minimal, used for early bring-up. |
| `facility.sdf` | (base shell) | 30x20 m facility shell: central E-W corridor with 6 rooms (NW/NC/NE north, SW/SC/SE south) off doorways. Robot home = (0,0) in the corridor. Base geometry reused by the gauge/inspection worlds. |
| `facility_gauges.sdf` | `gen_gauge_world.py` | `facility.sdf` plus 6 analog gauges on the south wall (Y=-10) at camera height, spaced for a strafe panorama sweep. |
| `facility_inspection.sdf` | `gen_inspection_world.py` | `facility.sdf` plus ~12 gauges of 5 types (PRESSURE/VOLTAGE/TEMPERATURE/CURRENT/FLOW) distributed across 5 rooms, forcing real navigation between targets. Some gauges read into the red danger zone for anomaly detection. |
| `inspection_arena.sdf` | `gen_inspection_arena.py` | Multi-room inspection facility reusing the `facility.sdf` shell, populated with Gazebo Fuel props (racks, drums, valves, pumps, electrical boxes, extinguishers, furniture, clutter), a self-contained fire (particle emitter), and a walking human actor. Per-room wall colours add visual variety for RGBD/RTAB-Map. |

All worlds load the same gz-sim system plugins: Physics, Sensors (ogre2 render engine), SceneBroadcaster, UserCommands, Contact, and Imu, with a 2 ms physics step.

Supporting assets, also under `worlds/`:

- `gauge_tex/`, `inspection_tex/`, `maze_tex/` - generated gauge-face PNGs plus a `*_groundtruth.json` manifest of each gauge's type/unit/range/true reading (used to score readings).
- `fire_tex/` - generated fire particle sprite (`puff.png`) and flame/smoke colour-over-lifetime ramps.
- `facility_map.npz` / `facility_map_viz.png` - rasterized occupancy grid of the facility walls (from `gen_map.py`).
- `zones.yaml` / `zones_viz.png` - zone segmentation reference for the facility map.

## Generator scripts

Located in `scripts/`. Run directly with `python3`. Note these scripts are **not** installed by CMake; run them from the source tree.

| Script | Output | Description |
|---|---|---|
| `gen_gauges.py [out_dir]` | `gauge_tex/gauge_NN.png` + `gauges_groundtruth.json` | Renders analog dial faces (270 deg sweep, major/minor ticks, numeric labels, red danger arc, needle at a known reading) and writes the ground-truth manifest. Default output `../worlds/gauge_tex`. Provides `draw_gauge()`, reused by the other gauge worlds. |
| `gen_gauge_world.py` | `facility_gauges.sdf` | Injects emissive gauge panels onto the facility south wall. Creates a no-space symlink `~/.go2_gauge_tex` -> `worlds/gauge_tex` and references that absolute path (gz cannot resolve texture paths through directories containing spaces). |
| `gen_inspection_world.py` | `facility_inspection.sdf`, `inspection_tex/`, `inspection_groundtruth.json` | Distributes ~12 gauges across 5 rooms of the facility shell; reuses `draw_gauge`. Uses no-space symlink `~/.go2_inspection_tex`. Additive and non-destructive to `facility.sdf`. |
| `gen_inspection_arena.py` | `inspection_arena.sdf`, `fire_tex/` | Builds the populated multi-room arena: Fuel-model props, a particle-emitter fire (with generated sprite/ramp textures), and a walking human actor. All props `<static>`. First launch downloads Fuel models to `~/.gz/fuel` (network required), then caches. |
| `gen_maze_world.py` | `maze.sdf`, `maze_tex/`, `maze_groundtruth.json` | Builds the 4-room maze; reuses `draw_gauge`. Uses no-space symlink `~/.go2_maze_tex`. |
| `gen_map.py [sdf]` | `facility_map.npz`, `facility_map_viz.png` | Rasterizes the facility SDF walls into a ROS-convention occupancy grid (`-1` unknown / `0` free / `100` occupied) with `RES=0.05`. A deterministic, noise-free stand-in for the live RTAB-Map `/map` when developing zone segmentation offline. Default input `facility.sdf`. |

Texture-path note: gz-sim cannot resolve a texture path that passes through a directory containing a space, and RGBA PNGs may be misread as transparent. The generators therefore flatten textures to RGB and expose them through a no-space symlink in `$HOME`, referenced by absolute path. **Re-run the world generators on each machine** to refresh the symlink and paths.

## Build & run

This package only installs `worlds/` (see `CMakeLists.txt`); generators are run manually from source.

```bash
# Build
cd go2-sim/go2_ws && colcon build --symlink-install --packages-select go2_worlds
source install/setup.bash

# (Re)generate worlds + textures on this machine (run from the package source tree)
cd src/go2_worlds/scripts
python3 gen_gauges.py
python3 gen_maze_world.py
python3 gen_gauge_world.py
python3 gen_inspection_world.py
python3 gen_inspection_arena.py
python3 gen_map.py

# Use a world via go2_bringup (this package has no launch files of its own)
ros2 launch go2_bringup sim_mapping.launch.py world:=maze.sdf headless:=false
ros2 launch go2_bringup sim_mapping.launch.py world:=inspection_arena.sdf headless:=false
```

## Dependencies

- **Build:** `ament_cmake`.
- **Generator scripts (Python 3):** `numpy`, `matplotlib` (gauge/fire textures), `opencv-python` (`cv2`, map rasterization).
- **Runtime:** Gazebo Harmonic (gz-sim) with the ogre2 render engine; `inspection_arena.sdf` additionally pulls OpenRobotics models from Gazebo Fuel on first launch (network required, then cached in `~/.gz/fuel`).
