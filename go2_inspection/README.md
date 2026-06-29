# go2_inspection — autonomous object inspection

Per-zone pipeline: **drive to a zone → sample a few viewpoints inside it → 360° in-place spin at each
while running live open-vocabulary YOLOE detection → project every detection onto the map via the depth
camera → de-duplicate (class + world position) → crop each unique object → per-zone + facility report and
annotated maps.** Detection + report only (no instrument reading). Built on the RTAB-Map + Nav2 + frontier
mapping stack (which it does **not** modify).

## Why viewpoint + spin (not wall-following)
A 360° spin from a few interior viewpoints sees walls **and** room-interior props (drums, pallets, crates,
the fire, a person) — wall-following only faces walls and misses everything in the middle. The spin is
slow (`spin_speed≈0.4 rad/s`) so detection runs continuously off a background thread without stop-and-shoot.

## Components
| Stage | Module | Role |
|---|---|---|
| zone segmentation | `go2_zones/zone_segmenter.py` | occupancy grid → zones (island-fill removes prop fragmentation; `nav_point` = deepest free point; quadrant `label`) |
| scan one zone | `zone_inspector` (ROS node) | viewpoint sampling + 360° spin + live YOLOE + depth→map projection + dedup + crops + report + `zone_map.png` |
| detection helpers | `detect_utils.py` | YOLOE open-vocab (`set_classes(names, get_text_pe(names))`), arena `PROMPTS`, contact sheet |
| report/plot helpers | `report_utils.py` | world↔pixel on the saved `.pgm`, zone/facility map plots, `report.md`/`report.csv` |
| mission orchestrator | `inspection_mission` | visit candidate zones → inspect each → return HOME → facility manifest + map + report |
| service layer | `mission_control_server` | ROS2 services (mapping / navigation / inspection / data) |
| MCP server | `mcp_mission_server` | one MCP tool per service (drive the sim in natural language) |

## Dependencies
- system ROS Jazzy: rclpy, nav2, tf2, opencv, numpy, scipy
- pip: `ultralytics` (YOLOE) + a CLIP text backend for open-vocab prompts
  (`pip install --user --break-system-packages "git+https://github.com/ultralytics/CLIP.git"`), and `fastmcp` for the MCP server.
- YOLOE weights: place `yoloe-11s-seg.pt` and point `YOLOE_WEIGHTS` at it (default `~/weights/yoloe-11s-seg.pt`).
  Without weights/CLIP the scan still runs (navigates + spins) and writes an empty result — it degrades gracefully.

## Run (sim)
```bash
# base stack: localization + static map_server + Nav2 (hazards present so they can be detected)
ros2 launch go2_bringup inspection_nav.launch.py world:=inspection_arena.sdf \
    map_yaml:=~/.go2_maps/facility_inspection_map.yaml actor:=true fire:=true headless:=true

export YOLOE_WEIGHTS=~/weights/yoloe-11s-seg.pt

# inspect ONE zone
ros2 run go2_inspection zone_inspector --ros-args -p use_sim_time:=true \
    -p zone_id:=zone_1 -p zones_file:=~/.go2_maps/facility_inspection_zones.yaml

# or the full mission (all candidate zones) + the service layer
ros2 launch go2_bringup mission_control.launch.py
ros2 launch go2_bringup mission.launch.py            # from HOME, every zone, then HOME
```

## Output (`~/gauges/<zone>/`)
- `crops/<id>.png` — one crop per unique object
- `objects.json` — deduped uniques: `{id, class, confidence, world:[x,y,z], crop, viewpoint, n_observations, localized}`
- `detections.json` — every accepted observation
- `objects_contact_sheet.png`, `zone_map.png` (objects plotted on the world map), `report.md`, `report.csv`

Facility rollup (`~/gauges/`): `facility_inspection_manifest.json`, `facility_map.png`, `facility_report.md`.
