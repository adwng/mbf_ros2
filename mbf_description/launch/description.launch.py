from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'mbf_description'
    package_share_dir = FindPackageShare(package_name)
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to run in simulation'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch Rviz or not'
    )
    
    rviz_path = PathJoinSubstitution(
        [package_share_dir, "rviz", "mbf.rviz"]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            package_share_dir,
            "xacro",
            "robot.xacro"
        ]),
        " use_sim:=",
        LaunchConfiguration("use_sim"),
    ])


    robot_description = {"robot_description": robot_description_content, 'use_sim_time': LaunchConfiguration('use_sim')}

    
    ros2_control_sim = PathJoinSubstitution(
        [package_share_dir, "config", "ros2_control_sim.yaml"]
    )

    ros2_control_real = PathJoinSubstitution(
        [package_share_dir, "config", "ros2_control.yaml"]
    )

    # Nodes
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description,],
    )

    control_node_sim = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control_sim],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    control_node_real = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control_real],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim")),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )


    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller"],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share_dir,
                'launch',
                'launch_gazebo.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )



    # Launch description
    return LaunchDescription([
        use_sim_arg,
        rviz_arg,
        robot_state_pub_node,
        control_node_sim,
        control_node_real,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz2,
        gazebo,
    ])
