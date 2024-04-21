"""Starts all the nodes to visualize one robot in rviz."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
    EqualsSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_jsp",
                default_value="gui",
                description="gui: use jsp_gui, jsp: use joint_state_publisher,\
                      none: no joint states published",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="true",
                description="true (default): start rviz, otherwise don't start rviz",
            ),
            DeclareLaunchArgument(
                name="color",
                default_value="purple",
                description="Color of nuturtle's body",
                choices=["red", "green", "blue", "purple", ""],
            ),
            SetLaunchConfiguration(
                name="rviz_color",
                value=[
                    FindPackageShare("nuturtle_description"),
                    TextSubstitution(text="/config/basic_"),
                    LaunchConfiguration("color"),
                    TextSubstitution(text=".rviz"),
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                namespace=LaunchConfiguration("color"),
                executable="joint_state_publisher_gui",
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "gui")),
            ),
            Node(
                package="joint_state_publisher",
                namespace=LaunchConfiguration("color"),
                executable="joint_state_publisher",
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "jsp")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=LaunchConfiguration("color"),
                parameters=[
                    {
                        "frame_prefix": PathJoinSubstitution(
                            [(LaunchConfiguration("color")), ""]
                        ),
                        "robot_description": Command(
                            [
                                TextSubstitution(text="xacro "),
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("nuturtle_description"),
                                        "urdf",
                                        "turtlebot3_burger.urdf.xacro",
                                    ]
                                ),
                                " color:=",
                                LaunchConfiguration("color"),
                            ]
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=LaunchConfiguration("color"),
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_color"),
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nuturtle_description"),
                            LaunchConfiguration("rviz_color"),
                        ]
                    ),
                ],
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_rviz"), "true")),
            ),
        ]
    )
