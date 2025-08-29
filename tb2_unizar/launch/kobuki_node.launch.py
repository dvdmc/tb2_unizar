# Launch file for Kobuki node with namespace support
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    tb2_unizar_dir = get_package_share_directory('tb2_unizar')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # Define remappings
    remappings = [('commands/velocity', 'cmd_vel'), ('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create launch configuration arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([tb2_unizar_dir, 'config', 'optitrack', 'kobuki_params.yaml']),
        description='Path to the YAML file with parameters for the Kobuki node'
    )


    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time
        },
        convert_types=True
    )


    # Create the Kobuki node
    kobuki_node = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        kobuki_node
    ])