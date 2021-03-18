from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument

import os

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = LaunchDescription()
    
    blueye_params = os.path.join(
        get_package_share_directory('blueye_ros2_interface'),
        'config',
        'ros2_interface_node_params.yaml'
        )
    
    
    blueye_node =  Node(
            namespace='blueye',
            name='ros2_interface_node',
            package='blueye_ros2_interface',            
            executable='blueye_interface',            
            output='screen',
            emulate_tty=True,
            parameters= [blueye_params],
            remappings=[]
        )
    
    ###################################################################
        
    gstreamer_node_params = os.path.join(
        get_package_share_directory('blueye_ros2_interface'),
        'config',
        'gstreamer_node_params.yaml'
        )
        
    gstreamer_remappings = [('image_raw', 'camera_img_raw'),
                  ('camera_info', 'camera_calibration_params')]
    
    gstreamer_node = Node (
            namespace='blueye',
            name='gstreamer_node',
            package='gscam',
            executable='gscam_main',
            output='screen',
            emulate_tty=True,
            parameters= [gstreamer_node_params],
            remappings=gstreamer_remappings
    
        )
    
    ###################################################################
    
    """joy_config = 'ps3'
    config_filepath = [ TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]
    DeclareLaunchArgument('joy_config', default_value='ps3'),
    DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
    DeclareLaunchArgument('config_filepath', default_value=[
    TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),
    """
    
    """ld.DeclareLaunchArgument('joy_config', default_value='ps3')
    ld.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    ld.DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, TextSubstitution(text='.config.yaml')])
    """
    
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
    
    rqt_image_view_node = Node (
            namespace='blueye',
            name='rqt_image_view_node',
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
            emulate_tty=True,
            #parameters= [],
            #remappings= []
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
    
    ###################################################################
    
    ld.add_action(blueye_node)
    ld.add_action(gstreamer_node)    
    ##ld.add_action(rqt_image_view_node)
    ld.add_action(rqt_gui_node)
    
    
    ld.add_action(joystick_node)
    #ld.add_action(joystick_teleop_node)
    
    
    
    
    return ld

