from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Whether to run in simulation'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description="use rviz2 or not"
    )

    descriptionPrefix = FindPackageShare("mbf_description")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                descriptionPrefix,
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'rviz': LaunchConfiguration('rviz')
        }.items()
    )

    imu_node = Node(
        package='imu',
        executable='imu_node',
        parameters=[{
            'port_name': '/dev/ttyUSB0',
            'rate': 60
        }],
        condition=UnlessCondition(
            LaunchConfiguration("use_sim")
        )
    )

    joy_node = Node(
        package="joy",
        executable="joy_node"
    )

    return LaunchDescription([
        use_sim_arg,
        rviz_arg,
        description,
        joy_node,
        imu_node,
    ])

