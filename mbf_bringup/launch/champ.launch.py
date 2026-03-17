from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    bringup_prefix = FindPackageShare("mbf_bringup")
    description_prefix = FindPackageShare("mbf_description")

    # Config files
    joints_config = PathJoinSubstitution([
        bringup_prefix, 'config', 'joints.yaml'
    ])

    gait_config = PathJoinSubstitution([
        bringup_prefix, 'config', 'gait.yaml'
    ])

    links_config = PathJoinSubstitution([
        bringup_prefix, 'config', 'links.yaml'
    ])

    control_config = PathJoinSubstitution([
        bringup_prefix, 'config', 'ctrl_param.yaml'
    ])

    description_path = PathJoinSubstitution([
        description_prefix, 'xacro', 'robot.xacro'
    ])

    mbf_control_node = Node(
        package='mbf_control',
        executable='mbf_control_node',
        output="screen",
        # prefix=['gdb -ex run --args'],
        parameters=[
            {"urdf": Command(['xacro ', description_path])},
            joints_config,
            links_config,
            gait_config,
            control_config,
        ]
    )

    return LaunchDescription([
        mbf_control_node
    ])