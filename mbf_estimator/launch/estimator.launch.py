"""Launch the body_estimator node.

Same pattern as mbf_se_bridge: expand mbf_description's xacro and pass
the resulting URDF in as 'robot_description' so Pinocchio can build its
model. This launch file only starts the estimator — pair it with
mbf_se_bridge (for contact wrenches) at a higher level.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mbf_estimator")
    default_cfg = os.path.join(pkg_share, "config", "estimator.yaml")

    cfg_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_cfg,
        description="Path to mbf_estimator estimator.yaml.",
    )
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Forwarded to xacro processing of mbf.xacro.",
    )

    mbf_desc = get_package_share_directory("mbf_description")
    xacro_path = os.path.join(mbf_desc, "xacro", "robot.xacro")

    robot_description = Command([
        "xacro ", xacro_path, " use_sim:=", LaunchConfiguration("use_sim"),
    ])

    body_estimator = Node(
        package="mbf_estimator",
        executable="body_estimator",
        name="body_estimator",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"robot_description": robot_description},
        ],
    )

    return LaunchDescription([
        cfg_arg,
        use_sim_arg,
        body_estimator,
    ])
