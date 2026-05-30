"""
M0609 + MoveIt + RViz demo launch.

Mobile base support: the world->base_link transform is parameterized via
xacro args (base_x, base_y, base_z). Pass `base_y:=0.30` to shift the robot
toward the side mirror — this expands mirror reach from ~18% to ~60%.

Default pose is base at world (0, 0, 0.85), the counter-wiping configuration.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, RegisterEventHandler, OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def _build_nodes(context, *args, **kwargs):
    base_x = LaunchConfiguration('base_x').perform(context)
    base_y = LaunchConfiguration('base_y').perform(context)
    base_z = LaunchConfiguration('base_z').perform(context)

    moveit_config = (
        MoveItConfigsBuilder("m0609", package_name="dsr_moveit_config_m0609")
        .robot_description(
            file_path="config/m0609.urdf.xacro",
            mappings={'base_x': base_x, 'base_y': base_y, 'base_z': base_z},
        )
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_full_config = os.path.join(
        get_package_share_directory("dsr_moveit_config_m0609"),
        "launch", "moveit.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("dsr_moveit_config_m0609"),
        "config", "ros2_controllers.yaml",
    )
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dsr_moveit_controller",
                   "--controller-manager", "/controller_manager"],
    )

    tuck_pose_sender = Node(
        package="arm_ik_service",
        executable="tuck_pose_sender",
        name="tuck_pose_sender",
        output="screen",
    )
    delayed_tuck = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[TimerAction(period=1.0, actions=[tuck_pose_sender])],
        )
    )

    return [
        robot_state_publisher,
        move_group,
        rviz,
        ros2_control,
        joint_state_broadcaster_spawner,
        dsr_moveit_controller_spawner,
        delayed_tuck,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('model', default_value='m0609',
                              description='Robot model'),
        DeclareLaunchArgument('base_x', default_value='0.0',
                              description='Robot base X in world frame'),
        DeclareLaunchArgument('base_y', default_value='0.0',
                              description='Robot base Y in world frame '
                                          '(0.30 for mirror task)'),
        DeclareLaunchArgument('base_z', default_value='0.85',
                              description='Robot base Z in world frame'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_build_nodes)])
