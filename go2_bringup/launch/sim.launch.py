"""Stage-1 sim bringup (compatibility wrapper -> go2_champ.launch.py).

The original ADR-003 "velocity base" model (urdf/go2.urdf) was superseded by the ADR-010
CHAMP walking base and its model file is no longer in the workspace. This launch now
delegates to go2_champ.launch.py so the familiar entry point still works and brings up the
supported walking Go2 in Gazebo Harmonic, publishing the mirrored real-Go2 topics (ADR-007):
  /utlidar/cloud_deskewed (PointCloud2), /utlidar/robot_odom (Odometry), /imu,
  /camera/image_raw, /cmd_vel (in), /clock, TF odom->base_link.

  ros2 launch go2_bringup sim.launch.py                            # GUI, lab world
  ros2 launch go2_bringup sim.launch.py headless:=true
  ros2 launch go2_bringup sim.launch.py world:=facility_inspection.sdf

For mapping/exploration or localization+Nav2, prefer the higher-level launchers:
  ros2 launch go2_bringup sim_mapping.launch.py      # fresh mapping (sim + RTAB-Map + Nav2)
  ros2 launch go2_bringup inspection_nav.launch.py   # localization + Nav2 on a saved map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    champ_launch = os.path.join(
        get_package_share_directory("go2_bringup"), "launch", "go2_champ.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="lab.sdf"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "champ",
                default_value="true",
                description="Run the CHAMP gait so the dog stands and walks from /cmd_vel.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "actor", default_value="true"
            ),  # inspection_arena: walking human (false=disable)
            DeclareLaunchArgument(
                "fire", default_value="true"
            ),  # inspection_arena: fire+smoke (false=keep drum only)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(champ_launch),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "headless": LaunchConfiguration("headless"),
                    "champ": LaunchConfiguration("champ"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "actor": LaunchConfiguration("actor"),
                    "fire": LaunchConfiguration("fire"),
                }.items(),
            ),
        ]
    )
