"""inspection_nav.launch.py -- facility-wide localization + Nav2 for the inspection mission.

map->odom localization has TWO modes, selected by `static_map_odom`:

  static_map_odom:=true  (DEFAULT -- SIM): publish a STATIC map->odom IDENTITY. The sim uses GROUND-TRUTH
    gz odometry and the saved map was built in that SAME frame starting from HOME, so map == odom (RTAB-Map's
    live localization correction here was only ~3 cm). This AVOIDS RTAB-Map localization's map->odom timing
    problem: it republishes map->odom at 20 Hz but only ADVANCES the stamp at its ~2 Hz processing rate, and
    under high sim real-time-factor the stamp STALLS >1 s behind sim-now. Nav2's controller then throws
    "Lookup would require extrapolation into the future" and ABORTS every goal (status 6). A static identity
    transform is always current, so Nav2 plans + drives reliably. Correct for the sim; NOT for a real robot.

  static_map_odom:=false (REAL ROBOT / no ground truth): RTAB-Map LOCALIZATION provides map->odom from the
    saved DB (the original design). Use this off-sim, where there is no ground-truth odom to lean on.

A static `map_server` serves the FULL saved facility grid on /map (Nav2's global costmap needs the full
map; rtabmap's own loc /map is only a local window). Nav2 (nav2_params_rtab tuning) plans + drives.
Robot starts at HOME (0,0). This is the navigation foundation the mission orchestrator (5c) drives.

  ros2 launch go2_bringup inspection_nav.launch.py world:=inspection_arena.sdf            # headless, static map->odom
  ros2 launch go2_bringup inspection_nav.launch.py world:=inspection_arena.sdf headless:=false
  ros2 launch go2_bringup inspection_nav.launch.py static_map_odom:=false                 # rtabmap localization
Requires the maps at ~/.go2_maps (symlink to go2-sim/maps) and FASTDDS_BUILTIN_TRANSPORTS=UDPv4.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _map_server(context, *a, **k):
    # expanduser here so a CLI-passed `map_yaml:=~/...` works (nav2_map_server uses raw fopen, no ~ expand).
    map_yaml = os.path.expanduser(LaunchConfiguration("map_yaml").perform(context))
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "yaml_filename": map_yaml,
                    "topic_name": "map",
                    "frame_id": "map",
                }
            ],
        )
    ]


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    static_mo = LaunchConfiguration("static_map_odom")
    fwd = {
        "world": world,
        "headless": headless,
        "actor": LaunchConfiguration("actor"),
        "fire": LaunchConfiguration("fire"),
        "spawn_x": LaunchConfiguration("spawn_x"),
        "spawn_y": LaunchConfiguration("spawn_y"),
        "spawn_yaw": LaunchConfiguration("spawn_yaw"),
    }

    # --- STATIC mode: sim only (gz ground-truth odom->base_link + sensors) + static map->odom identity ---
    sim_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "go2_champ.launch.py")),
        launch_arguments={**fwd, "champ": "true"}.items(),
        condition=IfCondition(static_mo),
    )
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom",
        output="screen",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],  # identity (map == ground-truth odom)
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(static_mo),
    )

    # --- RTAB-Map mode: sim + RTAB-Map localization (map->odom from saved DB) ---
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "rtabmap_slam.launch.py")),
        launch_arguments={
            **fwd,
            "localization": "true",
            "grid_topic": "/rtabmap/grid_map",
        }.items(),
        condition=UnlessCondition(static_mo),
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": os.path.join(pkg, "config", "nav2_params_rtab.yaml"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("world", default_value="inspection_arena.sdf"),
            DeclareLaunchArgument(
                "static_map_odom", default_value="true"
            ),  # sim: static identity map->odom
            DeclareLaunchArgument(
                "map_yaml",
                default_value=os.path.expanduser("~/.go2_maps/facility_inspection_map.yaml"),
            ),
            DeclareLaunchArgument("actor", default_value="true"),
            DeclareLaunchArgument("fire", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            sim_only,
            static_map_odom,
            slam,
            # static /map up early (map_server path expanduser'd via OpaqueFunction)
            TimerAction(period=5.0, actions=[OpaqueFunction(function=_map_server), map_lifecycle]),
            # Nav2: static map->odom is up immediately, so start Nav2 once the sim + CHAMP EKF settle (~30 s).
            TimerAction(period=30.0, actions=[nav2], condition=IfCondition(static_mo)),
            # RTAB-Map mode needs ~85 s for localization to come up before Nav2.
            TimerAction(period=85.0, actions=[nav2], condition=UnlessCondition(static_mo)),
        ]
    )
