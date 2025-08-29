# This file is inspired by: https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/bringup_launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import (
    Node,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    Command,
)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Paths to bringup and custom package
    tb2_unizar_dir = FindPackageShare("tb2_unizar")

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the robot"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true"
    )

    declare_urdf_file_cmd = DeclareLaunchArgument(
        "urdf_file",
        default_value=PathJoinSubstitution([tb2_unizar_dir, "urdf", "turtlebot_unizar_hokuyo_top.urdf.xacro"]),
        description="URDF file for the robot"
    )

    # Get the namespace
    namespace = LaunchConfiguration('namespace')

    # Locate description
    robot_description = Command(["xacro ", LaunchConfiguration("urdf_file")])

    remappings=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # Create the robot state publisher node
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_description": ParameterValue(robot_description, value_type=str)
        }],
        remappings=remappings
    )

    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_urdf_file_cmd,
            start_robot_state_publisher_cmd,
        ]
    )
