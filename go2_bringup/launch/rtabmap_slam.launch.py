"""RTAB-Map graph SLAM (localization + 3D mapping) for the Go2 -- see ADR-015.

A separate, parallel stack to the slam_toolbox + frontier 2D stack (it does not touch that stack).
Fuses the 4D LiDAR (/utlidar/cloud_filtered) + RGB camera (/camera) + accurate odom (/odom) into one
memory-managed graph-SLAM node that publishes:
  - map->odom TF          (localization; replaces slam_toolbox here)
  - /map                  (2D occupancy grid, projected from the LiDAR -- for Nav2 + frontier_explorer)
  - /rtabmap/cloud_map    (assembled 3D point cloud, RGB-coloured)
  - /rtabmap/mapGraph     (pose graph: nodes + odometry links + loop-closure links)
  - /rtabmap/mapData      (full graph data)

  ros2 launch go2_bringup rtabmap_slam.launch.py world:=facility.sdf                 # mapping (GUI sim)
  ros2 launch go2_bringup rtabmap_slam.launch.py world:=facility.sdf headless:=true  # headless gz
  ros2 launch go2_bringup rtabmap_slam.launch.py localization:=true                  # localize on saved DB

Sim-agnostic: identical topics on the real Go2 (see ADR-002/007); Orin-friendly (DetectionRate 2Hz,
lean ICP).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    headless = LaunchConfiguration("headless")
    with_sim = LaunchConfiguration("with_sim")
    localization = LaunchConfiguration("localization")
    continue_map = LaunchConfiguration("continue_map")  # resume + EXTEND a saved ~/.ros/rtabmap.db

    # The walking Go2 + gz sensors (4D LiDAR, RGB camera, IMU) + accurate odom (CHAMP+gz EKF).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "go2_champ.launch.py")),
        # 'world' MUST be forwarded -- without it go2_champ falls back to its default lab.sdf, which would
        # spawn the robot in lab while it localizes against the maze/facility DB+map (Nav2 goals then
        # unreachable and the robot never moves). Default stays lab.sdf, so callers that omit world are
        # unaffected.
        launch_arguments={
            "headless": headless,
            "champ": "true",
            "world": LaunchConfiguration("world"),
            "actor": LaunchConfiguration("actor"),
            "fire": LaunchConfiguration("fire"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
        }.items(),
        condition=IfCondition(with_sim),
    )

    # Remove the dog's own body from the L1 cloud (so RTAB-Map's ICP + grid aren't polluted).
    self_filter = Node(
        package="go2_exploration",
        executable="self_filter",
        name="self_filter",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # 2D /scan for Nav2's local obstacle layer (Phase 2). RTAB-Map builds the global grid itself.
    p2l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[("cloud_in", "/utlidar/cloud_filtered"), ("scan", "/scan")],
        parameters=[
            os.path.join(pkg, "config", "pointcloud_to_laserscan.yaml"),
            {"use_sim_time": True},
        ],
    )
    # Pre-fuse the RGBD camera (rgb + registered depth + camera_info) into ONE /rtabmap/rgbd_image, so
    # RTAB-Map sees the camera as a single optional input (can't stall LiDAR+odom node insertion). The
    # rgb+depth share one gz rgbd_camera stamp -> exact sync (approx_sync=false). COLOUR ONLY: this feeds
    # rtabmap's coloured cloud_map, never its grid/registration (see Kp/MaxFeatures=-1 + Grid/FromDepth).
    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        namespace="rtabmap",
        output="screen",
        parameters=[{"use_sim_time": True, "approx_sync": False, "qos": 1}],
        remappings=[
            ("rgb/image", "/camera/image_raw"),
            ("depth/image", "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/camera_info"),
        ],
    )
    rtabmap_params = {
        "use_sim_time": True,
        "frame_id": "base_link",
        "map_frame_id": "map",
        # LIDAR-DRIVEN SLAM + COLOUR-ONLY RGBD. The pose graph, registration, loop closure AND the 2D grid
        # are ALL still driven purely by the 3D LiDAR + the accurate fused /odom (exactly as before). The
        # RGBD camera is fed in ONLY to colour the assembled 3D cloud (/rtabmap/cloud_map); it is structurally
        # barred from SLAM by Kp/MaxFeatures=-1 (no visual loop closure -> the repetitive-facility teleport
        # cannot recur) + Reg/Strategy=1 (ICP registration) + Grid/FromDepth=false (camera is never a grid
        # source). rgb+depth are pre-fused by an rgbd_sync node into /rtabmap/rgbd_image (one optional input
        # that can't stall LiDAR+odom node insertion), so the 2D grid update rate is unchanged.
        "subscribe_rgbd": False,  # Camera contributes nothing to this lidar-only SLAM
        #   (Kp/MaxFeatures=-1, Grid/Sensor=0, Grid/FromDepth=false); it
        #   only coloured cloud_map. Feeding the 640x480 RGBD through
        #   rgbd_sync into rtabmap's approx_sync burned CPU that starved
        #   rtabmap's /clock callback, so its now() lagged and the map->odom
        #   TF (stamped now()+tf_tolerance on a 20 Hz thread) went ~1 s
        #   stale -> Nav2 "Transform data too old" aborted goals mid-path.
        #   RGB stays on /camera/* for inspection; /camera/points still
        #   feeds the Nav2 costmap directly.
        "subscribe_rgb": False,  # Required for the decouple to take effect: subscribe_depth
        "subscribe_depth": False,  #   defaults True, so with only subscribe_rgbd=False rtabmap falls
        #   back to pulling raw /rtabmap/{rgb,depth}/image + rgb/camera_info
        #   into its approx_sync -- a more fragile camera coupling than the
        #   single pre-synced rgbd_image. Both False => rtabmap syncs only
        #   scan_cloud + odom (pure lidar SLAM, zero camera dependency).
        "subscribe_scan_cloud": True,  # the 4D LiDAR (3D registration + 2D grid projection)
        "approx_sync": True,  # LiDAR 20Hz / cam 30Hz / odom 50Hz aren't time-synced
        "approx_sync_max_interval": 0.1,  # cap so a late camera frame can't stall scan+odom insertion
        "sync_queue_size": 8,  # A shallow queue keeps sync robust at 20/30/50 Hz while favouring
        #   fresh frames: a deeper queue lets rtabmap pick an old but
        #   time-consistent (scan+rgbd+odom) set over the newest, adding
        #   map->odom staleness on top of the detection-rate floor.
        "wait_for_transform": 0.15,  # In sim the odom->base_link TF leads the sensors by milliseconds,
        #   so a long wait only delays the processed stamp that advances
        #   map->odom; 0.15 s still covers startup transients.
        "qos_image": 1,
        "qos_camera_info": 1,
        "qos_scan": 1,
        "qos_odom": 1,
        # --- registration + loop closure (LiDAR geometry only; RGB never contributes) ---
        "Kp/MaxFeatures": "-1",  # disable bag-of-words -> NO visual loop closure (teleport-proof)
        "Reg/Strategy": "1",  # ICP (strong geometric registration from the 3D LiDAR)
        "Reg/Force3DoF": "true",  # ground robot: lock z/roll/pitch (prevents drift out of plane)
        "Icp/VoxelSize": "0.05",
        "Icp/PointToPlane": "true",
        "Icp/MaxCorrespondenceDistance": "0.3",
        "Icp/Epsilon": "0.001",
        "RGBD/ProximityBySpace": "true",  # LiDAR-geometry loop closure (sim walls are low-texture)
        "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/AngularUpdate": "0.05",  # add a node every 0.05 rad / 0.05 m of motion
        "RGBD/LinearUpdate": "0.05",
        "Vis/MinInliers": "12",  # (moot in pure-lidar mode; no visual loop closure)
        # --- 2D occupancy grid from the 3D LiDAR cloud (config from the Go2-Inspector reference) ---
        # Grid/Sensor "0" builds the grid from the LiDAR SCAN_CLOUD. "1" would build it from the depth
        # camera, but subscribe_depth=false, leaving no grid source -> /map all-unknown/wall-only. "0" plus
        # RayTracing carves real FREE space from each lidar ray -> /map gets free/occupied/unknown, so the
        # frontier can run DIRECTLY on /map (the reference architecture) with no Nav2-costmap workaround.
        "Grid/Sensor": "0",
        "Grid/FromDepth": "false",  # camera can NEVER be a grid source; grid stays from LiDAR (Grid/Sensor=0)
        "Grid/RayTracing": "true",  # carve free space: sensor->obstacle rays mark cells FREE
        "Grid/3D": "false",  # rtabmap does the LIGHT 2D grid + graph; octomap_server does the 3D
        "Grid/NormalsSegmentation": "false",  # flat-height ground split (robust on sparse lidar) -- matches ref
        "RGBD/CreateOccupancyGrid": "true",
        "RGBD/OptimizeMaxError": "3.0",  # tolerate odom-only links (few loop closures in low-texture sim)
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "5.0",  # match the reference (L1 reliable range for grid projection)
        "Grid/MaxGroundHeight": "0.15",  # z below this = floor = FREE; above = obstacle. The legged Go2
        # bobs/pitches as it walks, so ground returns momentarily rise
        # above the cutoff and get stamped as phantom obstacles in the 2D
        # grid; 0.15 m absorbs that body motion. (Cost: real obstacles
        # shorter than 0.15 m aren't in the GLOBAL grid; Nav2's LOCAL
        # costmap still sees them via the /scan band.)
        "Grid/MaxObstacleHeight": "1.2",  # 3D-LiDAR points ABOVE this z (m, base_link) are NOT stamped as
        # obstacles in the 2D grid. A ground robot can't be blocked by
        # anything that doesn't reach near the floor, so high returns
        # (rising smoke, ceiling, tall shelf-tops) would otherwise pollute
        # /map. Walls still map from their lower 1.2 m. The fire/smoke
        # PARTICLES are also made LiDAR-transparent at the source
        # (particle_scatter_ratio=0 in inspection_arena.sdf), so the flame
        # column never enters the cloud at all.
        # --- occupancy-grid node behaviour (Go2-Inspector reference) ---
        "map_always_update": True,  # refresh the grid every cycle, not only on new graph nodes
        "map_empty_ray_tracing": False,  # do NOT carve free PAST the last return: our sparse 32-ring
        # lidar has gaps, and free-past-walls created phantom frontiers
        # beyond the facility walls. Grid/RayTracing still carves free
        # up to each obstacle (safe). Walls stay solid in the grid.
        # --- memory (Orin-friendly) ---
        "Mem/STMSize": "30",
        "Rtabmap/DetectionRate": "2.0",  # 2 Hz graph update. map->odom is broadcast on a separate 20 Hz
        #   thread (stamped now()+tf_tolerance), so DetectionRate sets only
        #   how often the correction VALUE refreshes, not the TF stamp.
        #   Kept low to cut rtabmap CPU, since map->odom lag stems from
        #   rtabmap's /clock starving under load.
        # --- dense RGB-coloured 3D cloud (/rtabmap/cloud_map) assembly. Independent of Grid/* (which stays
        #     2D + LiDAR); these only bound the appearance cloud built from the RGBD keyframes. ---
        "cloud_voxel_size": 0.05,  # 5cm voxel downsample of the published coloured cloud
        "cloud_max_depth": 6.0,  # ignore depth beyond 6 m (sim depth noise grows with range)
        "cloud_min_depth": 0.3,
        "cloud_decimation": 2,  # 320x240 effective -> lighter assembly, still well-coloured
        "cloud_noise_filtering_radius": 0.05,
        "cloud_noise_filtering_min_neighbors": 3,
    }
    rtabmap_remap = [
        ("scan_cloud", "/utlidar/cloud_filtered"),
        # ("rgbd_image", "/rtabmap/rgbd_image"), # camera decoupled (subscribe_rgbd=False) to free CPU
        ("odom", "/odom"),
        # RTAB-Map's 2D grid topic is 'map'. Default -> /map (mapping/exploration). For the MISSION we set
        # grid_topic:=/rtabmap/grid_map so a STATIC map_server owns /map (full facility), while RTAB-Map
        # localization only provides map->odom + its local grid elsewhere (its localization /map is partial).
        ("map", LaunchConfiguration("grid_topic")),
    ]
    map_params = dict(rtabmap_params)
    map_params["Mem/IncrementalMemory"] = "true"  # SLAM
    loc_params = dict(rtabmap_params)
    loc_params["Mem/IncrementalMemory"] = "false"
    loc_params["Mem/InitWMWithAllNodes"] = "true"  # localize
    # The robot ALWAYS (re)spawns at HOME = the map origin (mapping started there), so ASSUME the start pose
    # is the map origin instead of doing a GLOBAL relocalization -- which, in a near-symmetric world (the
    # maze), can snap to a ROTATED match so the static /map and the live localized frame appear misaligned.
    # Localization-mode ONLY: SLAM (-d) + continue modes never set this, so mapping is unaffected.
    loc_params["RGBD/StartAtOrigin"] = "true"
    # CONTINUE: keep mapping (IncrementalMemory stays true) but LOAD all nodes of the existing DB into
    # working memory so the FULL prior map is active + republished, then extend it. No -d (don't wipe).
    cont_params = dict(map_params)
    cont_params["Mem/InitWMWithAllNodes"] = "true"

    # fresh mapping fires only when NOT localizing AND NOT continuing (so -d never wipes a resumed DB)
    fresh_cond = IfCondition(
        PythonExpression(["'", localization, "' == 'false' and '", continue_map, "' == 'false'"])
    )
    rtabmap_map = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[map_params],
        remappings=rtabmap_remap,
        arguments=[
            "-d",
            "--ros-args",
            "--log-level",
            "rtabmap.rtabmap:=warn",
            "--log-level",
            "rtabmap:=warn",
        ],
        # -d: fresh DB (mapping). --log-level rtabmap.rtabmap:=warn silences the per-cycle "rtabmap (N): Rate=..."
        # INFO stats spam (it is a direct ROS_INFO on the node logger, so only a node log-level arg hides it);
        # WARN/ERROR are preserved. To see full rtabmap INFO again, change warn to info here (and on cont/loc).
        condition=fresh_cond,
    )
    rtabmap_cont = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[cont_params],
        remappings=rtabmap_remap,
        arguments=[
            "--ros-args",
            "--log-level",
            "rtabmap.rtabmap:=warn",
            "--log-level",
            "rtabmap:=warn",
        ],
        # NO -d (load+EXTEND existing ~/.ros/rtabmap.db); --log-level silences the per-cycle stats spam.
        condition=IfCondition(continue_map),
    )
    rtabmap_loc = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[loc_params],
        remappings=rtabmap_remap,
        arguments=[
            "--ros-args",
            "--log-level",
            "rtabmap.rtabmap:=warn",
            "--log-level",
            "rtabmap:=warn",
        ],
        # --log-level silences the per-cycle stats spam (localization mode).
        condition=IfCondition(localization),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("with_sim", default_value="true"),
            DeclareLaunchArgument("world", default_value="lab.sdf"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("continue_map", default_value="false"),
            DeclareLaunchArgument("grid_topic", default_value="/map"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "actor", default_value="true"
            ),  # forwarded to go2_champ (inspection_arena)
            DeclareLaunchArgument(
                "fire", default_value="true"
            ),  # forwarded to go2_champ (inspection_arena)
            sim,
            self_filter,
            p2l,
            # Start RTAB-Map AFTER the sim + CHAMP EKF are up, so its first frames have a stable
            # odom->base_link TF (otherwise it loses odometry at startup and stops adding nodes).
            # rgbd_sync is intentionally NOT launched: the camera is decoupled from rtabmap (subscribe_rgbd=False)
            # to free the CPU that was starving rtabmap's clock and lagging map->odom. The rgbd_sync Node defined
            # above is therefore currently unused.
            TimerAction(period=14.0, actions=[rtabmap_map, rtabmap_cont, rtabmap_loc]),
            # 3D map: octomap_server on the SAME LiDAR cloud, built in RTAB-Map's loop-closure-corrected
            # 'map' frame (rtabmap's internal dense cloud_map assembly was unreliable here). Starts after
            # rtabmap publishes map->odom. -> /occupied_cells_vis_array (colored 3D voxels).
            TimerAction(
                period=20.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg, "launch", "octomap.launch.py")
                        ),
                        launch_arguments={"use_sim_time": "true"}.items(),
                    ),
                ],
            ),
        ]
    )
