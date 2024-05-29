import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import tempfile

colors = ["cyan", "magenta", "yellow", "red", "green", "blue", "orange", "brown", "white"]

def adjust_slam_params(slam_params, sim_speed_multiplier):
    """
    Adjust slam_toolbox parameters based on sim_speed_multiplier.
    """
    adjusted_params = slam_params.copy()
    adjusted_params['transform_publish_period'] = slam_params['transform_publish_period'] / sim_speed_multiplier
    adjusted_params['map_update_interval'] = slam_params['map_update_interval'] / sim_speed_multiplier
    adjusted_params['minimum_time_interval'] = slam_params['minimum_time_interval'] / sim_speed_multiplier
    adjusted_params['transform_timeout'] = slam_params['transform_timeout'] / sim_speed_multiplier
    adjusted_params['tf_buffer_duration'] = slam_params['tf_buffer_duration'] / sim_speed_multiplier
    return adjusted_params

def load_slam_params(file_path):
    """
    Load slam_toolbox parameters from the YAML file.
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def launch_setup(context):

    iterations = int(LaunchConfiguration('iterations').perform(context))
    sim_speed_multiplier = float(LaunchConfiguration('sim_speed_multiplier').perform(context))

    units = []

    for i in range(iterations):

        # Load and adjust slam parameters
        slam_params_file_path = os.path.join(get_package_share_directory('multislam'), 'config', colors[i] + "_map_online_async_fake.yaml")
        slam_params = load_slam_params(slam_params_file_path)
        adjusted_slam_params = adjust_slam_params(slam_params['/**/slam_toolbox']['ros__parameters'], sim_speed_multiplier)

        # Save adjusted parameters to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.yaml') as temp_file:
            temp_slam_params_file = temp_file.name
            adjusted_slam_params_yaml = {'/**/slam_toolbox': {'ros__parameters': adjusted_slam_params}}
            yaml.dump(adjusted_slam_params_yaml, temp_file)

        units.append(
            GroupAction(
                actions=[
                    PushRosNamespace(colors[i]),

                    SetRemap(src='/map', dst='/' + colors[i] + '/map'),

                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
                        launch_arguments={
                            "slam_params_file": temp_slam_params_file,
                            "use_sim_time": "true",
                        }.items(),
                    ),
                ]
            )
        )

        units.append(
            Node(package="tf2_ros",
                 executable="static_transform_publisher",
                 arguments=["0", "0", "0", "0", "0", "0", "multisim/world", colors[i] + "/map"])
        )

        units.append(
            Node(package="tf2_ros",
                 executable="static_transform_publisher",
                 arguments=["0", "0", "0", "0", "0", "0", colors[i] + "/map", colors[i] + "/odom"])
        )

    return units

def generate_launch_description():

    # Arguments for number of iterations and simulation speed multiplier
    launch_args = [
        DeclareLaunchArgument(
            name="iterations",
            default_value="3",
            description="Number of robots",
        ),
        DeclareLaunchArgument(
            name="sim_speed_multiplier",
            default_value="1",
            description="Simulation speed multiplier",
        ),
    ]

    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)

    ld.add_action(opfunc)

    return ld
