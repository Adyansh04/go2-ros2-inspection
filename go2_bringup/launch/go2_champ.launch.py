"""Real walking Go2 in Gazebo Harmonic via CHAMP + gz_ros2_control (ADR-010).

  ros2 launch go2_bringup go2_champ.launch.py                 # GUI, lab world
  ros2 launch go2_bringup go2_champ.launch.py headless:=true

Layers (staggered): gz sim + RSP -> spawn + bridge -> controllers -> champ gait.
The Go2 then walks from /cmd_vel; sensors publish the mirrored real-Go2 topics (ADR-007).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *a, **k):
    desc = get_package_share_directory("go2_description")
    cfg = get_package_share_directory("go2_config")
    worlds = get_package_share_directory("go2_worlds")
    bringup = get_package_share_directory("go2_bringup")
    champ_bringup = get_package_share_directory("champ_bringup")

    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    world = LaunchConfiguration("world").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)  # spawn pose, map frame (metres)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)  # spawn heading (rad); 0 = +X
    world_path = os.path.join(worlds, "worlds", world)
    # actor/fire toggles: inspection_arena.sdf ships a walking <actor> and a <model name="hazard_fire">.
    # The moving actor complicates SLAM and the fire is a distraction during mapping, so allow disabling
    # either via launch args. We strip the relevant SDF block(s) into a temp world and launch that.
    # Default (both true) -> original world file is used unchanged (no effect on maze/facility worlds).
    actor_on = LaunchConfiguration("actor").perform(context).lower() in ("1", "true", "yes")
    fire_on = LaunchConfiguration("fire").perform(context).lower() in ("1", "true", "yes")
    if not actor_on or not fire_on:
        import re as _re, tempfile as _tf

        try:
            _txt = open(world_path).read()
            if not actor_on:  # remove the walking human
                _txt = _re.sub(r"[ \t]*<actor\b.*?</actor>\s*", "", _txt, flags=_re.DOTALL)
            if not fire_on:  # remove ONLY the flames+smoke model; the drum (sw_drum) is a separate include and stays
                _txt = _re.sub(
                    r'[ \t]*<model name="hazard_fire".*?</model>\s*', "", _txt, flags=_re.DOTALL
                )
            _wfd, world_path = _tf.mkstemp(prefix="arena_", suffix=".sdf")
            with os.fdopen(_wfd, "w") as _wf:
                _wf.write(_txt)
            print(f"[go2_champ] world toggles: actor={actor_on} fire={fire_on} -> {world_path}")
        except Exception as _e:
            print(f"[go2_champ] actor/fire world-preprocess failed ({_e}); using original world")
            world_path = os.path.join(worlds, "worlds", world)
    xacro_file = os.path.join(desc, "xacro", "robot_gz.xacro")
    # Process the xacro to a URDF string. Use a subprocess (file as a single argv element) so a space in
    # the workspace path can't break Command()'s shlex split. Fail loudly if xacro errors or yields nothing
    # (otherwise RSP silently starts with an empty robot_description and TF/joints break with a cryptic error).
    import subprocess, tempfile

    _proc = subprocess.run(["xacro", xacro_file], capture_output=True, text=True)
    if _proc.returncode != 0 or not _proc.stdout.strip():
        raise RuntimeError(
            f"xacro failed for {xacro_file} (returncode={_proc.returncode}).\nstderr:\n{_proc.stderr}"
        )
    robot_description = _proc.stdout
    # champ_bringup re-xacro-processes a file PATH, so also write the URDF to a UNIQUE temp file
    # (a fixed /tmp/go2_gz.urdf race-clobbers across concurrent launches / leaves stale copies).
    _fd, urdf_out = tempfile.mkstemp(prefix="go2_gz_", suffix=".urdf")
    with os.fdopen(_fd, "w") as _f:
        _f.write(robot_description)

    gz_flags = "-s --headless-rendering -r -v3" if headless else "-r -v3"
    # gz must find the gz_ros2_control system plugin (libgz_ros2_control-system.so) — it lives
    # in the ROS lib dir, which is NOT on gz's default system-plugin search path.
    ros_lib = os.path.join(
        os.environ.get("AMENT_PREFIX_PATH", "/opt/ros/jazzy").split(":")[0], "lib"
    )
    sys_plugin_path = os.pathsep.join(
        p
        for p in ["/opt/ros/jazzy/lib", ros_lib, os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
        if p
    )
    gz = ExecuteProcess(
        cmd=["gz", "sim", *gz_flags.split(), world_path],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": f"{worlds}:{desc}",
            "GZ_SIM_SYSTEM_PLUGIN_PATH": sys_plugin_path,
            # Force gz/Ogre2 to use NVIDIA's EGL ICD for offscreen sensor (camera)
            # rendering. Without this the EGL loader falls back to llvmpipe software
            # rendering, which is slow enough to starve other CPU-bound nodes. The
            # PRIME vars route rendering to the discrete NVIDIA GPU.
            "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            "__NV_PRIME_RENDER_OFFLOAD": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        },
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "go2",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            "0.30",
            "-Y",
            spawn_yaw,
        ],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(bringup, "config", "ros_gz_bridge_champ.yaml"),
                "use_sim_time": True,
            }
        ],
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_states_controller", "-c", "/controller_manager"],
    )
    jtc = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_group_effort_controller", "-c", "/controller_manager"],
    )

    do_champ = LaunchConfiguration("champ").perform(context).lower() in ("1", "true", "yes")
    champ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(champ_bringup, "launch", "bringup.launch.py")),
        launch_arguments={
            "description_path": urdf_out,
            "joints_map_path": os.path.join(cfg, "config/joints/joints.yaml"),
            "links_map_path": os.path.join(cfg, "config/links/links.yaml"),
            "gait_config_path": os.path.join(cfg, "config/gait/gait.yaml"),
            "use_sim_time": "true",
            "robot_name": "go2",
            "gazebo": "true",
            "lite": "false",
            "rviz": "false",
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "joint_hardware_connected": "false",
            "publish_foot_contacts": "true",
        }.items(),
    )

    # Tight staggering minimises the window where the dog is spawned but no controller is holding
    # it (limp legs -> collapse). gz up -> spawn -> controllers active -> gait, ~6.5s total.
    actions = [
        gz,
        rsp,
        TimerAction(period=3.0, actions=[spawn, bridge]),
        TimerAction(period=5.0, actions=[jsb]),
        TimerAction(period=6.0, actions=[jtc]),
    ]
    if do_champ:
        actions.append(
            TimerAction(period=8.0, actions=[champ])
        )  # gait controller holds stand, then walks
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world", default_value="lab.sdf"),
            DeclareLaunchArgument("champ", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "actor",
                default_value="true",
                description="inspection_arena: spawn the walking human actor (false=disable, e.g. for SLAM)",
            ),
            DeclareLaunchArgument(
                "fire",
                default_value="true",
                description="inspection_arena: enable the fire+smoke particle emitters (false=keep drum, no flames/smoke)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
