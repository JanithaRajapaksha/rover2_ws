import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():

    package_name = 'mecanum_control'

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )

    mecanum_node = Node(
        package="mecanum_control",
        executable="serial_mecanum_node",
        name="mecanum_node_serial",
        output="screen"
    )

    # Launch them all!
    return LaunchDescription([
        twist_mux,
        mecanum_node
    ])