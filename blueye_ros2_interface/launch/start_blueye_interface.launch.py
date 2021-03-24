from launch import LaunchDescription
from launch_ros.actions import Node


from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    pkg_dir = get_package_share_directory('blueye_ros2_interface')
    
    launch_bttm_side = IncludeLaunchDescription( \
        PythonLaunchDescriptionSource( pkg_dir + \
        '/launch/start_blueye_interface_bttm_side.launch.py')) 
    
    launch_top_side = IncludeLaunchDescription( \
        PythonLaunchDescriptionSource( pkg_dir + \
        '/launch/start_blueye_interface_top_side.launch.py'))     
        
    ld.add_action(launch_bttm_side)
    ld.add_action(launch_top_side)
    return ld
