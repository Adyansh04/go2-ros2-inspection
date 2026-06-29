import os
import launch_ros
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # EE26 cleanup: champ_config + champ_description (generic non-Go2 clone) removed. joints/links/gait
    # come from go2_config via launch args; the default model resolves the Go2 description from
    # go2_description. Localization is now gz ground-truth (OdometryPublisher -> /odom + odom->base_link
    # TF, bridged in go2_champ.launch.py), so the robot_localization EKFs + champ state_estimation are no
    # longer launched here. go2_champ always passes description_path + rviz:=false, so these defaults only
    # apply if bringup.launch.py is ever run standalone.
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_gz.xacro")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true")
    declare_description_path = DeclareLaunchArgument(
        name="description_path", default_value=default_model_path,
        description="Absolute path to robot urdf/xacro file")
    declare_rviz_path = DeclareLaunchArgument(
        name="rviz_path", default_value="", description="Absolute path to rviz file")
    declare_joints_map_path = DeclareLaunchArgument(
        name="joints_map_path", default_value="", description="Absolute path to joints map file")
    declare_links_map_path = DeclareLaunchArgument(
        name="links_map_path", default_value="", description="Absolute path to links map file")
    declare_gait_config_path = DeclareLaunchArgument(
        name="gait_config_path", default_value="", description="Absolute path to gait config file")
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz")
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="/", description="Robot name")
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite")
    declare_gazebo = DeclareLaunchArgument(
        "gazebo", default_value="false", description="If in gazebo")
    declare_joint_controller_topic = DeclareLaunchArgument(
        "joint_controller_topic",
        default_value="joint_group_effort_controller/joint_trajectory",
        description="Joint controller topic")
    declare_hardware_connected = DeclareLaunchArgument(
        "joint_hardware_connected", default_value="false",
        description="Whether hardware is connected")
    declare_publish_joint_control = DeclareLaunchArgument(
        "publish_joint_control", default_value="true", description="Publish joint control")
    declare_publish_joint_states = DeclareLaunchArgument(
        "publish_joint_states", default_value="true", description="Publish joint states")
    declare_publish_foot_contacts = DeclareLaunchArgument(
        "publish_foot_contacts", default_value="true", description="Publish foot contacts")

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"gazebo": LaunchConfiguration("gazebo")},
            {"publish_joint_states": LaunchConfiguration("publish_joint_states")},
            {"publish_joint_control": LaunchConfiguration("publish_joint_control")},
            {"publish_foot_contacts": LaunchConfiguration("publish_foot_contacts")},
            {"joint_controller_topic": LaunchConfiguration("joint_controller_topic")},
            {"urdf": ParameterValue(Command(['xacro ', LaunchConfiguration('description_path')]), value_type=str)},
            LaunchConfiguration('joints_map_path'),
            LaunchConfiguration('links_map_path'),
            LaunchConfiguration('gait_config_path'),
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    rviz2 = Node(
        package='rviz2', namespace='', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_path")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_description_path,
        declare_rviz_path,
        declare_joints_map_path,
        declare_links_map_path,
        declare_gait_config_path,
        declare_rviz,
        declare_robot_name,
        declare_lite,
        declare_gazebo,
        declare_joint_controller_topic,
        declare_hardware_connected,
        declare_publish_joint_control,
        declare_publish_joint_states,
        declare_publish_foot_contacts,
        # RSP runs in go2_champ.launch.py; localization is gz ground-truth (no EKF / state_estimation).
        quadruped_controller_node,
        rviz2,
    ])
