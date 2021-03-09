from launch import LaunchDescription
from launch_ros.actions import Node

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
    
    
    ld.add_action(blueye_node)
    ld.add_action(gstreamer_node)
    #ld.add_action(rqt_image_view_node)
    ld.add_action(rqt_gui_node)
    
    return ld

