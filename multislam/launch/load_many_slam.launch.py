import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, EqualsSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import random

colors = ["cyan", "magenta", "yellow", "red", "green", "blue", "orange", "brown", "white"]

def launch_setup(context):

    iterations = int(LaunchConfiguration('iterations').perform(context)) # Here you'll get the runtime config value

    units = []

    for i in range(iterations):

        units.append(
            GroupAction(
                actions=[
                    PushRosNamespace(colors[i]),

                    SetRemap(src='/map',dst= '/' + colors[i] + '/map'),

                    IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
                    launch_arguments={
                        "slam_params_file": PathJoinSubstitution([FindPackageShare('multislam'), 'config', colors[i] + "_map_online_async_fake.yaml"]),  
                        "use_sim_time": "true",  
                        }.items(),
                    ),
                ]
            )
            
        )

        units.append(
            Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "multisim/world", colors[i] + "/map"])
        )

        units.append(
            Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", colors[i] + "/map", colors[i] + "/odom"])
        )
        
    return units

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