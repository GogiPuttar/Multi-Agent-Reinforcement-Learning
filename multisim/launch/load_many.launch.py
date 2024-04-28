import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, EqualsSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import random

colors = ["cyan", "magenta", "yellow", "red", "green", "blue", "orange", "brown", "white"]

def launch_setup(context):

    iterations = int(LaunchConfiguration('iterations').perform(context)) # Here you'll get the runtime config value

    nodes = []

    for i in range(iterations):

        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('nuturtle_description'), '/launch/load_one.launch.py']),
                launch_arguments={
                    "use_jsp": "gui",  # Use jsp_gui for joint state publishing
                    "use_rviz": "false",  # Start RViz
                    "color": colors[i],  # Assign color to each turtle
                }.items(),
            )
        )
        
    return nodes

def generate_launch_description():

    # Argument for number of iterations
    launch_args = [
        DeclareLaunchArgument(
            name="iterations",
            default_value="3",
            description="Number of robots",
        )
    ]

    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)

    return ld