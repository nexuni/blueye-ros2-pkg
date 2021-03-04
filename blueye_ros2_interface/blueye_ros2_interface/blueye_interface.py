#!/usr/bin/env python3

from blueye.sdk import Pioneer

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from blueye_ros2_msgs.msg import BlueyeCameraParams


import math
import sys


class BlueyeInterface(Node):

    def velocity_ref_callback(self, data):
        surge_val = 0.25 * data.linear.x
        sway_val = 0.25 * data.linear.y
        heave_val = 0.25 * data.linear.z
        yaw_val = 0.25 * data.angular.z

        # self.drone.motion.send_thruster_setpoint(surge=surge_val, sway=sway_val, heave=heave_val, yaw=yaw_val)
        if not self.is_simulation:
            self.drone.motion.surge = surge_val
            self.drone.motion.sway = yaw_val
            self.drone.motion.yaw = sway_val
            self.drone.motion.heave = heave_val


    def lights_lvl_ref_callback(self, data):
        if not self.is_simulation:
            self.drone.lights = data


    def declare_node_parameters(self):
        print("Declaring ROS parameters")
        self.declare_parameter('rate', 10)
        self.declare_parameter('is_simulation', False)
    
    
    def get_ros_params(self):        
        # Setting ROS parameters
        self.rate = self.get_parameter('rate').get_parameter_value().double_value #ParameterType.msg
        self.is_simulation = self.get_parameter('is_simulation').get_parameter_value().bool_value
        

    def initialize_timer(self):
        print("Initializing timer")
        self.timer_period = 1.0/self.rate
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback)


    def timer_callback(self):
        self.get_ros_params()
        self.publish_all_blueye_variables()


    def publish_all_blueye_variables(self):
        # Publishing ROV params and variables to ROS topics
        if not self.is_simulation:
            # Publishing depth and orientation into a Pose msg
            msg = Pose()
            msg.position.x = 0.0
            msg.position.y = 0.0
            msg.position.z = float(self.drone.depth)  # TODO: Check unit
            # Make sure the quaternion is valid and normalized
            roll = float(list(self.drone.pose.values())[0])  # TODO: Check unit
            pitch = float(list(self.drone.pose.values())
                          [1])  # TODO: Check unit
            yaw = float(list(self.drone.pose.values())[2])  # TODO: Check unit
            # TODO: Check if drone.pose angles are in rad or degrees
            x, y, z, w = self.euler_to_quaternion(roll, pitch, yaw)
            msg.orientation.x = float(x)
            msg.orientation.y = float(y)
            msg.orientation.z = float(z)
            msg.orientation.w = float(w)
            self.pose_pub.publish(msg)

            # Publish surge, sway, heave and yaw rate velocities as a Twist msg
            msg = Twist()
            msg.linear.x = float(self.drone.motion.surge)  # TODO: Check unit
            msg.linear.y = float(self.drone.motion.sway)  # TODO: Check unit
            msg.linear.z = float(self.drone.motion.heave)  # TODO: Check unit
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            # TODO: Check if drone.motin.yaw is in rad or degrees
            msg.angular.z = float(self.drone.motion.yaw)
            self.velocity_pub.publish(msg)

            # Publishing light level as an Int32 msg
            msg = Int32()  # TODO: Check unit and range
            msg.data = int(self.drone.lights)
            self.lights_lvl_pub.publish(msg)

            # Publishing connection status
            msg = Bool()
            msg.data = self.drone.connection_established
            self.connected_status_pub.publish(msg)

            # Publishing active auto mode
            auto_depth_active = self.drone.motion.auto_depth_active
            auto_heading_active = self.drone.motion.auto_heading_active
            msg = Int32()
            # Coding auto depth and heading modes to 0-4 depending on
            # 00, 01, 10, 11 active state for depth-heading
            msg.data = 2*auto_depth_active + 1*auto_heading_active
            self.auto_mode_pub.publish(msg)

            # Publishing ROV camera parameters
            msg = BlueyeCameraParams()
            msg.bitrate = self.drone.camera.bitrate  # TODO: Check range
            msg.exposure = self.drone.camera.exposure  # TODO: Check range
            msg.frames_per_second = self.drone.camera.framerate  # TODO: Check range
            msg.is_recording = self.drone.camera.is_recording  # TODO: Check range
            msg.hue = self.drone.camera.hue  # TODO: Check range
            msg.recording_time = self.drone.camera.record_time  # TODO: Check range
            msg.resolution = self.drone.camera.resolution  # TODO: Check range
            msg.tilt_angle = self.drone.camera.tilt.angle  # TODO: Check range
            msg.white_balance = self.drone.camera.whitebalance     # TODO: Check range
            self.camera_params_pub.publish(msg)


    def euler_to_quaternion(self, roll, pitch, yaw):  # yaw (Z), pitch (Y), roll (X)
        # Abbreviations for the various angular functions
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w


    def initialize_subscribers(self):
        print("Initializing ROS subscribers")
        # Initialize ROS subscribers to ROV variables' reference - ROV set topics
        self.create_subscription(
            Twist, "velocity_ref", self.velocity_ref_callback, 10)
        self.create_subscription(
            Int32, "lights_lvl_ref", self.lights_lvl_ref_callback, 10)


    def initialize_publishers(self):
        print("Initializing ROS publishers")
        # Initialize ROS publishers of ROV variables - ROV get topics
        self.pose_pub = self.create_publisher(Pose, "pose", 10)
        self.velocity_pub = self.create_publisher(
            Twist, "velocity", 10)
        self.lights_lvl_pub = self.create_publisher(
            Int32, "lights_lvl", 10)
        self.connected_status_pub = self.create_publisher(
            Bool, "connected_status", 10)
        self.auto_mode_pub = self.create_publisher(
            Int32, "auto_mode", 10)  # 0 - none, 1 - aDepth, 2 - aHeading
        self.camera_params_pub = self.create_publisher(
            BlueyeCameraParams, "camera_params", 10)


    def initialize_blueye_connection(self):
        print("Initializing Blueye connection")
        if not self.is_simulation:
            try:
                self.drone = Pioneer()
                print("Drone successfully instantiated.")
            except:
                print("Could not instantiate Pioneer.")
                pass
        else:
            print("Drone not instantiated.")


    def __init__(self):
        print("Initializing blueye_interface class instance.")

        super().__init__('blueye_interface')
        
        self.declare_node_parameters()
        self.get_ros_params()

        self.initialize_timer()      

        # rclpy.init()
        #self.node = rclpy.create_node('blueye_interface')
        #self.node_rate = self.node.create_rate(self.rate)

        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_blueye_connection()


def main(args=None):
    print("Started")
    rclpy.init(args=args)

    try:
        interface = BlueyeInterface()
        rclpy.spin(interface)
        # interface.run()
    except:
        print("Exception caught!")
        e = sys.exc_info()[0]
        write_to_page("<p>Error: %s</p>" % e)
        pass

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
