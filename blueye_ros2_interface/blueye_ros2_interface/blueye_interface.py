#!/usr/bin/env python3

from blueye.sdk import Drone

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

    def velocity_ref_callback(self, msg):
        surge_val = 0.25 * msg.linear.x
        sway_val = 0.25 * msg.linear.y
        heave_val = 0.25 * msg.linear.z
        yaw_val = 0.25 * msg.angular.z

        # self.drone.motion.send_thruster_setpoint(surge=surge_val, sway=sway_val, heave=heave_val, yaw=yaw_val)
        if not self.IS_SIMULATION:
            self.drone.motion.surge = surge_val
            self.drone.motion.sway = yaw_val
            self.drone.motion.yaw = sway_val
            self.drone.motion.heave = heave_val
        else:
            return

    def lights_lvl_ref_callback(self, msg):
        if not self.IS_SIMULATION:
            self.drone.lights = msg
        else:
            return

    def auto_mode_ref_callback(self, msg):
        if not self.IS_SIMULATION:
            # 00, 01, 10, 11 active state for depth-heading
            mode = msg.data
            if (mode == 0):
                self.drone.motion.auto_depth_active = 0
                self.drone.motion.auto_heading_active = 0
            elif (mode == 1):
                self.drone.motion.auto_depth_active = 0
                self.drone.motion.auto_heading_active = 1
            elif (mode == 2):
                self.drone.motion.auto_depth_active = 1
                self.drone.motion.auto_heading_active = 0
            elif (mode == 3):
                self.drone.motion.auto_depth_active = 1
                self.drone.motion.auto_heading_active = 1
        else:
            return

    def camera_params_ref_callback(self, msg):
        if not self.IS_SIMULATION:

            if (msg.bitrate >= self.CAMERA_BITRATE_MIN and msg.bitrate <= self.CAMERA_BITRATE_MAX):
                print("Bitrate in range. Setting.")
                self.drone.camera.bitrate = msg.bitrate
            elif (msg.bitrate < self.CAMERA_BITRATE_MIN):
                print("Desired bitrate below limit, setting to CAMERA_BITRATE_MIN")
                self.drone.camera.bitrate = self.CAMERA_BITRATE_MIN
            elif (msg.bitrate > self.CAMERA_BITRATE_MAX):
                print("Desired bitrate above limit, setting to CAMERA_BITRATE_MAX")
                self.drone.camera.bitrate = self.CAMERA_BITRATE_MAX

            if msg.exposure >= self.CAMERA_EXPOSURE_MIN and msg.exposure <= self.CAMERA_EXPOSURE_MAX:
                self.drone.camera.exposure = msg.exposure
            elif msg.exposure < self.CAMERA_EXPOSURE_MIN:
                print("Desired exposure below limit, setting to CAMERA_EXPOSURE_MIN")
                self.drone.camera.exposure = self.CAMERA_EXPOSURE_MIN
            elif msg.exposure > self.CAMERA_EXPOSURE_MAX:
                print("Desired exposure above limit, setting to CAMERA_EXPOSURE_MAX")
                self.drone.camera.exposure = self.CAMERA_EXPOSURE_MAX

            if msg.frames_per_second in self.CAMERA_FRAMERATE_VALUES:
                self.drone.camera.framerate = msg.frames_per_second
            elif msg.frames_per_second < self.CAMERA_FRAMERATE_VALUES[0]:
                print("Desired framerate below limit, setting to CAMERA_FRAMERATE_MIN")
                self.drone.camera.framerate = self.CAMERA_FRAMERATE_VALUES[0]
            elif msg.frames_per_second > self.CAMERA_FRAMERATE_VALUES[1]:
                print("Desired framerate above limit, setting to CAMERA_FRAMERATE_MAX")
                self.drone.camera.framerate = self.CAMERA_FRAMERATE_VALUES[1]

            self.drone.camera.is_recording = msg.is_recording

            if msg.hue >= self.CAMERA_HUE_MIN and msg.hue <= self.CAMERA_HUE_MAX:
                self.drone.camera.hue = msg.hue
            elif msg.hue < self.CAMERA_HUE_MIN:
                print("Desired hue below limit, setting to CAMERA_HUE_MIN")
                self.drone.camera.hue = self.CAMERA_HUE_MIN
            elif msg.hue > self.CAMERA_HUE_MAX:
                print("Desired hue above limit, setting to CAMERA_HUE_MAX")
                self.drone.camera.hue = self.CAMERA_HUE_MAX

            if (msg.resolution in self.CAMERA_RESOLUTION_VALUES):
                self.drone.camera.resolution = msg.resolution
                print(msg.resolution)
            elif (msg.resolution < self.CAMERA_RESOLUTION_VALUES[0]):
                print(
                    "Desired resolution below limit, setting to CAMERA_RESOLUTION_VALUES_MIN")
                self.drone.camera.resolution = self.CAMERA_RESOLUTION_VALUES[0]
            elif (msg.resolution > self.CAMERA_RESOLUTION_VALUES[2]):
                print(
                    "Desired resolution above limit, setting to CAMERA_RESOLUTION_VALUES_MAX")
                self.drone.camera.resolution = self.CAMERA_RESOLUTION_VALUES[2]
            else:
                print(
                    "Desired resolution in the 480-1080 range but not standard, setting to CAMERA_RESOLUTION_VALUES_MAX")
                self.drone.camera.resolution = self.CAMERA_RESOLUTION_VALUES[2]

            if (msg.whitebalance == -1 or (msg.whitebalance >= self.CAMERA_WHITEBALANCE_MIN and msg.whitebalance <= self.CAMERA_WHITEBALANCE_MAX)):
                print("White balance in range. Setting.")
                self.drone.camera.whitebalance = msg.whitebalance
            elif (msg.whitebalance < self.CAMERA_WHITEBALANCE_MIN):
                print(
                    "Desired whitebalance below limit, setting to CAMERA_WHITEBALANCE_MIN")
                self.drone.camera.whitebalance = self.CAMERA_WHITEBALANCE_MIN
            elif (msg.whitebalance > self.CAMERA_WHITEBALANCE_MAX):
                print(
                    "Desired whitebalance above limit, setting to CAMERA_WHITEBALANCE_MAX")
                self.drone.camera.whitebalance = self.CAMERA_WHITEBALANCE_MAX

            if (msg.tilt_speed_ref >= self.CAMERA_TILT_SPEED_MIN and msg.tilt_speed_ref <= self.CAMERA_TILT_SPEED_MAX):
                self.drone.camera.tilt.set_speed(msg.tilt_speed_ref)
            elif (msg.tilt_speed < self.CAMERA_TILT_SPEED_MIN):
                print("Desired hue below limit, setting to CAMERA_TILT_SPEED_MIN")
                self.drone.camera.tilt.set_speed(CAMERA_TILT_SPEED_MIN)
            elif (msg.tilt_speed > self.CAMERA_TILT_SPEED_MAX):
                print("Desired hue above limit, setting to CAMERA_TILT_SPEED_MAX")
                self.drone.camera.tilt.set_speed(self.CAMERA_TILT_SPEED_MAX)
        else:
            return

    def declare_node_parameters(self):
        print("Declaring ROS parameters")
        # ROS params
        self.declare_parameter('rate', 10)
        self.declare_parameter('is_simulation', False)

        # Blueye params
        # Camera params
        #self.declare_parameter('camera_bitrate', 6000000)
        self.declare_parameter('camera_bitrate_min', 1000000)
        self.declare_parameter('camera_bitrate_max', 16000000)

        #self.declare_parameter('camera_exposure', -1)
        self.declare_parameter('camera_exposure_min', 1)
        self.declare_parameter('camera_exposure_max', 5000)

        #self.declare_parameter('camera_framerate', 25)
        self.declare_parameter('camera_framerate_values', [25, 30])

        #self.declare_parameter('camera_hue', 0)
        self.declare_parameter('camera_hue_min', -40)
        self.declare_parameter('camera_hue_max', 40)

        #self.declare_parameter('camera_resolution', 1080)
        self.declare_parameter('camera_resolution_values', [480, 720, 1080])

        #self.declare_parameter('camera_whitebalance', -1)
        self.declare_parameter('camera_whitebalance_min', 2800)
        self.declare_parameter('camera_whitebalance_max', 9300)

        #self.declare_parameter('camera_tilt_speed', 0)
        self.declare_parameter('camera_tilt_speed_min', -1)
        self.declare_parameter('camera_tilt_speed_max', 1)

    def get_ros_params(self):
        # Setting ROS parameters
        self.RATE = self.get_parameter(
            'rate').get_parameter_value().double_value  # ParameterType.msg
        self.IS_SIMULATION = self.get_parameter(
            'is_simulation').get_parameter_value().bool_value

        # Setting Blueye parameters
        # Camera params
        # self.camera_bitrate = self.get_parameter(
        #   'camera_bitrate').get_parameter_value().integer_value
        self.CAMERA_BITRATE_MIN = self.get_parameter(
            'camera_bitrate_min').get_parameter_value().integer_value
        self.CAMERA_BITRATE_MAX = self.get_parameter(
            'camera_bitrate_max').get_parameter_value().integer_value

        # self.camera_exposure = self.get_parameter(
        #   'camera_exposure').get_parameter_value().integer_value
        self.CAMERA_EXPOSURE_MIN = self.get_parameter(
            'camera_exposure_min').get_parameter_value().integer_value
        self.CAMERA_EXPOSURE_MAX = self.get_parameter(
            'camera_exposure_max').get_parameter_value().integer_value

        # self.camera_framerate = self.get_parameter(
        #   'camera_framerate').get_parameter_value().integer_value
        self.CAMERA_FRAMERATE_VALUES = self.get_parameter(
            'camera_framerate_values').get_parameter_value().integer_array_value

        # self.camera_hue = self.get_parameter(
        #   'camera_hue').get_parameter_value().integer_value
        self.CAMERA_HUE_MIN = self.get_parameter(
            'camera_hue_min').get_parameter_value().integer_value
        self.CAMERA_HUE_MAX = self.get_parameter(
            'camera_hue_max').get_parameter_value().integer_value

        # self.camera_resolution = self.get_parameter(
        #   'camera_resolution').get_parameter_value().integer_value
        self.CAMERA_RESOLUTION_VALUES = self.get_parameter(
            'camera_resolution_values').get_parameter_value().integer_array_value

        # self.camera_whitebalance = self.get_parameter(
        #   'camera_whitebalance').get_parameter_value().integer_value
        self.CAMERA_WHITEBALANCE_MIN = self.get_parameter(
            'camera_whitebalance_min').get_parameter_value().integer_value
        self.CAMERA_WHITEBALANCE_MAX = self.get_parameter(
            'camera_whitebalance_max').get_parameter_value().integer_value

        # self.camera_tilt_speed = self.get_parameter(
        #   'camera_tilt_speed').get_parameter_value().integer_value
        self.CAMERA_TILT_SPEED_MIN = self.get_parameter(
            'camera_tilt_speed_min').get_parameter_value().integer_value
        self.CAMERA_TILT_SPEED_MAX = self.get_parameter(
            'camera_tilt_speed_max').get_parameter_value().integer_value

    def set_blueye_params(self):
        return

    def initialize_timer(self):
        print("Initializing timer")
        self.timer_period = 1.0/self.RATE
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback)

    def timer_callback(self):
        # self.get_ros_params() # Blueye params values should be changed
        # through topic publishing and not ROS2 node param change
        self.set_blueye_params()
        self.publish_all_blueye_variables()

    def publish_all_blueye_variables(self):
        # Publishing ROV params and variables to ROS topics
        if not self.IS_SIMULATION:
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
            msg.tilt_speed_ref = 0
            msg.whitebalance = self.drone.camera.whitebalance     # TODO: Check range
            self.camera_params_pub.publish(msg)
        else:
            return

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
        self.create_subscription(
            Int32, "auto_mode_ref", self.auto_mode_ref_callback, 10)
        self.create_subscription(
            BlueyeCameraParams, "camera_params_ref", self.camera_params_ref_callback, 10)

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
            Int32, "auto_mode", 10)  # 00, 01, 10, 11 active state for depth-heading
        self.camera_params_pub = self.create_publisher(
            BlueyeCameraParams, "camera_params", 10)

    def initialize_blueye_connection(self):
        print("Initializing Blueye connection")
        if not self.IS_SIMULATION:
            try:
                self.drone = Drone()
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
        # self.node = rclpy.create_node('blueye_interface')
        # self.node_rate = self.node.create_rate(self.RATE)

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
