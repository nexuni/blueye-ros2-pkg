import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
              
    joystick_node_params = os.path.join(
        get_package_share_directory('blueye_ros2_interface'),
        'config',
        'joystick_node_params.yaml'
        )

    joystick_node = Node (
            namespace='blueye',
            name='blueye_joystic_node',
            package='joy',
            executable='joy_node',
            output='screen',
            emulate_tty=True,
            #arguments = [ '--perspective-file', rqt_gui_node_perspective  ] 
            parameters=[ joystick_node_params ]
            #remappings= joystick_remappings
        )
    
    joystick_teleop_node = Node(
            namespace = 'blueye',
            name='blueye_teleop_twist_joy_node',
            package='teleop_twist_joy', 
            executable='teleop_node',
            output='screen',
            emulate_tty=True,
            parameters=[] #[config_filepath]
        )
    
    ###################################################################
        
    rqt_gui_node_perspective =  os.path.join(
        get_package_share_directory('blueye_ros2_interface'),
        'config',
        'blueye_rqt_gui.perspective'
    )  
    
    rqt_gui_node = Node (
            namespace='blueye',
            name='gui',
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            emulate_tty=True,
            arguments = [ '--perspective-file', rqt_gui_node_perspective  ] 
            #parameters= [],
            #remappings= []
        )       
            
    ld.add_action(rqt_gui_node)     
    ld.add_action(joystick_node)
    #ld.add_action(joystick_teleop_node)    
    return ld

