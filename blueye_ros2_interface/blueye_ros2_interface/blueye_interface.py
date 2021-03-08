#!/usr/bin/env python3

from blueye.sdk import Drone

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from blueye_ros2_msgs.msg import BlueyeCameraParams

import math
import sys


class BlueyeInterface(Node):

    def thruster_force_norm_ref_callback(self, msg):
        surge_val = msg.linear.x
        sway_val = msg.linear.y
        heave_val = msg.linear.z
        yaw_val = msg.angular.z

        if not self.IS_SIMULATION:
            # TODO: Check if and why yaw and sway are switched
            if (surge_val >= self.VELOCITY_FORCE_MIN and surge_val <= self.VELOCITY_FORCE_MAX):
                print("Surge force in range. Setting")
                self.drone.motion.surge = surge_val
            elif (surge_val < self.VELOCITY_FORCE_MIN):
                print("Surge force is below range. Setting to VELOCITY_FORCE_MIN")
                self.drone.motion.surge = self.VELOCITY_FORCE_MIN
            elif (surge_val > self.VELOCITY_FORCE_MAX):
                print("Surge force is below range. Setting to VELOCITY_FORCE_MAX")
                self.drone.motion.surge = self.VELOCITY_FORCE_MAX

            if (sway_val >= self.VELOCITY_FORCE_MIN and sway_val <= self.VELOCITY_FORCE_MAX):
                print("Sway force in range. Setting")
                self.drone.motion.sway = sway_val  # TODO: Check if .yaw
            elif (sway_val < self.VELOCITY_FORCE_MIN):
                print("Sway force is below range. Setting to VELOCITY_FORCE_MIN")
                self.drone.motion.sway = self.VELOCITY_FORCE_MIN
            elif (sway_val > self.VELOCITY_FORCE_MAX):
                print("Sway force is below range. Setting to VELOCITY_FORCE_MAX")
                self.drone.motion.sway = self.VELOCITY_FORCE_MAX

            if (heave_val >= self.VELOCITY_FORCE_MIN and heave_val <= self.VELOCITY_FORCE_MAX):
                print("Heave force in range. Setting")
                self.drone.motion.heave = heave_val
            elif (heave_val < self.VELOCITY_FORCE_MIN):
                print("Heave force is below range. Setting to VELOCITY_FORCE_MIN")
                self.drone.motion.heave = self.VELOCITY_FORCE_MIN
            elif (heave_val > self.VELOCITY_FORCE_MAX):
                print("Heave force is below range. Setting to VELOCITY_FORCE_MAX")
                self.drone.motion.heave = self.VELOCITY_FORCE_MAX

            if (yaw_val >= self.VELOCITY_FORCE_MIN and yaw_val <= self.VELOCITY_FORCE_MAX):
                print("Yaw moment force in range. Setting")
                self.drone.motion.yaw = yaw_val  # TODO: Check if .sway
            elif (yaw_val < self.VELOCITY_FORCE_MIN):
                print("Yaw moment  is below range. Setting to VELOCITY_FORCE_MIN")
                self.drone.motion.yaw = self.VELOCITY_FORCE_MIN
            elif (yaw_val > self.VELOCITY_FORCE_MAX):
                print("Yaw moment  is below range. Setting to VELOCITY_FORCE_MAX")
                self.drone.motion.yaw = self.VELOCITY_FORCE_MAX

            #self.drone.motion.surge = surge_val
            #self.drone.motion.sway = yaw_val
            #self.drone.motion.yaw = sway_val
            #self.drone.motion.heave = heave_val
        else:
            return

    def lights_lvl_ref_callback(self, msg):
        lights_ref = msg.data
        if not self.IS_SIMULATION:
            if (lights_ref >= self.LIGHTS_LEVEL_MIN and lights_ref <= self.LIGHTS_LEVEL_MAX):
                print("Desired light level is in range. Setting.")
                self.drone.lights = lights_ref
            elif (lights_ref < self.LIGHTS_LEVEL_MIN):
                print("Desired light level  below range. Setting to LIGHTS_LEVEL_MIN.")
                self.drone.lights = self.LIGHTS_LEVEL_MIN
            elif (lights_ref > self.LIGHTS_LEVEL_MAX):
                print("Desired light level  above range. Setting to LIGHTS_LEVEL_MAX.")
                self.drone.lights = self.LIGHTS_LEVEL_MAX
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
                print("Invalid auto-mode. Keeping the current mode.")
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
                print("Exposure in range. Setting.")
                self.drone.camera.exposure = msg.exposure
            elif msg.exposure < self.CAMERA_EXPOSURE_MIN:
                print("Desired exposure below limit, setting to CAMERA_EXPOSURE_MIN")
                self.drone.camera.exposure = self.CAMERA_EXPOSURE_MIN
            elif msg.exposure > self.CAMERA_EXPOSURE_MAX:
                print("Desired exposure above limit, setting to CAMERA_EXPOSURE_MAX")
                self.drone.camera.exposure = self.CAMERA_EXPOSURE_MAX

            if msg.frames_per_second in self.CAMERA_FRAMERATE_VALUES:
                print("Framerate in range. Setting.")
                self.drone.camera.framerate = msg.frames_per_second
            elif msg.frames_per_second < self.CAMERA_FRAMERATE_VALUES[0]:
                print("Desired framerate below limit, setting to CAMERA_FRAMERATE_MIN")
                self.drone.camera.framerate = self.CAMERA_FRAMERATE_VALUES[0]
            elif msg.frames_per_second > self.CAMERA_FRAMERATE_VALUES[1]:
                print("Desired framerate above limit, setting to CAMERA_FRAMERATE_MAX")
                self.drone.camera.framerate = self.CAMERA_FRAMERATE_VALUES[1]

            self.drone.camera.is_recording = msg.is_recording

            if msg.hue >= self.CAMERA_HUE_MIN and msg.hue <= self.CAMERA_HUE_MAX:
                print("Hue in range. Setting.")
                self.drone.camera.hue = msg.hue
            elif msg.hue < self.CAMERA_HUE_MIN:
                print("Desired hue below limit, setting to CAMERA_HUE_MIN")
                self.drone.camera.hue = self.CAMERA_HUE_MIN
            elif msg.hue > self.CAMERA_HUE_MAX:
                print("Desired hue above limit, setting to CAMERA_HUE_MAX")
                self.drone.camera.hue = self.CAMERA_HUE_MAX

            if (msg.resolution in self.CAMERA_RESOLUTION_VALUES):
                print("Resolution in range. Setting.")
                self.drone.camera.resolution = msg.resolution
                # print(msg.resolution)
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
                print("Tilt speed in range. Setting.")
                self.drone.camera.tilt.set_speed(msg.tilt_speed_ref)
            elif (msg.tilt_speed < self.CAMERA_TILT_SPEED_MIN):
                print("Desired tilt speed below limit, setting to CAMERA_TILT_SPEED_MIN")
                self.drone.camera.tilt.set_speed(CAMERA_TILT_SPEED_MIN)
            elif (msg.tilt_speed > self.CAMERA_TILT_SPEED_MAX):
                print("Desired tilt speed above limit, setting to CAMERA_TILT_SPEED_MAX")
                self.drone.camera.tilt.set_speed(self.CAMERA_TILT_SPEED_MAX)
        else:
            return

    def boost_gain_ref_callback(self, msg):
        boost_gain = msg.data
        if not self.IS_SIMULATION:
            if (boost_gain >= self.BOOST_GAIN_MIN and boost_gain <= self.BOOST_GAIN_MAX):
                print("Boost gain in range. Setting.")
                self.drone.motion.boost = boost_gain
            elif (boost_gain < self.BOOST_GAIN_MIN):
                print("Boost gain below range. Setting to BOOST_GAIN_MIN.")
                self.drone.motion.boost = self.BOOST_GAIN_MIN
            elif (boost_gain > self.BOOST_GAIN_MAX):
                print("Boost gain above range. Setting to BOOST_GAIN_MAX.")
                self.drone.motion.boost = self.BOOST_GAIN_MAX

    def slow_gain_ref_callback(self, msg):
        slow_gain = msg.data
        if not self.IS_SIMULATION:
            if (slow_gain >= self.SLOW_GAIN_MIN and slow_gain <= self.SLOW_GAIN_MAX):
                print("Slow gain in range. Setting.")
                self.drone.motion.slow = slow_gain
            elif (slow_gain < self.SLOW_GAIN_MIN):
                print("Slow gain below range. Setting to SLOW_GAIN_MIN.")
                self.drone.motion.slow = self.SLOW_GAIN_MIN
            elif (slow_gain > self.SLOW_GAIN_MAX):
                print("Slow gain above range. Setting to SLOW_GAIN_MAX.")
                self.drone.motion.slow = self.SLOW_GAIN_MAX

    def water_density_ref_callback(self, msg):
        water_density = msg.data
        if not self.IS_SIMULATION:
            if water_density >= self.WATER_DENSITY_MIN and water_density <= self.WATER_DENSITY_MAX:
                print("Water density in range. Setting.")
                self.drone.config.water_density = water_density
            elif water_density < self.WATER_DENSITY_MIN:
                print("Water density below range. Setting to WATER_DENSITY_MIN.")
                self.drone.config.water_density = self.WATER_DENSITY_MIN
            elif water_density > self.WATER_DENSITY_MAX:
                print("Water density above range. Setting to WATER_DENSITY_MAX.")
                self.drone.config.water_density = self.WATER_DENSITY_MAX

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

        # Lights level params
        self.declare_parameter('lights_level_min', 0)
        self.declare_parameter('lights_level_max', 255)

        # Reference force for velocities limits
        self.declare_parameter('velocity_force_min', -1.0)
        self.declare_parameter('velocity_force_max', 1.0)

        # Boost gain limits
        self.declare_parameter('boost_gain_min', 0.0)
        self.declare_parameter('boost_gain_max', 1.0)

        # Slow gain limits
        self.declare_parameter('slow_gain_min', 0.0)
        self.declare_parameter('slow_gain_max', 1.0)

        # Water density limits
        self.declare_parameter('water_density_min', 997)
        self.declare_parameter('water_density_max', 1240)

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

        # Light level params
        self.LIGHTS_LEVEL_MIN = self.get_parameter(
            'lights_level_min').get_parameter_value().integer_value
        self.LIGHTS_LEVEL_MAX = self.get_parameter(
            'lights_level_max').get_parameter_value().integer_value

        # Reference forces' limits
        self.VELOCITY_FORCE_MIN = self.get_parameter(
            'velocity_force_min').get_parameter_value().integer_value
        self.VELOCITY_FORCE_MAX = self.get_parameter(
            'velocity_force_max').get_parameter_value().integer_value

        # Boost gain limits
        self.BOOST_GAIN_MIN = self.get_parameter(
            'boost_gain_min').get_parameter_value().double_value
        self.BOOST_GAIN_MAX = self.get_parameter(
            'boost_gain_max').get_parameter_value().double_value

        # Slow gain limits
        self.SLOW_GAIN_MIN = self.get_parameter(
            'slow_gain_min').get_parameter_value().double_value
        self.SLOW_GAIN_MAX = self.get_parameter(
            'slow_gain_max').get_parameter_value().double_value

        # Water density limits
        self.WATER_DENSITY_MIN = self.get_parameter(
            'water_density_min').get_parameter_value().integer_value
        self.WATER_DENSITY_MAX = self.get_parameter(
            'water_density_max').get_parameter_value().integer_value

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
            # conversion of depth from [mm]to [m]
            msg.position.z = float(self.drone.depth) / 1000.0
            # Make sure the quaternion is valid and normalized
            # conversion of roll, pith and yaw from [degrees]to [rad]
            roll = math.radians(float(list(self.drone.pose.values())[0]))
            pitch = math.radians(float(list(self.drone.pose.values())[1]))
            yaw = math.radians(float(list(self.drone.pose.values())[2]))
            x, y, z, w = self.euler_to_quaternion(roll, pitch, yaw)
            msg.orientation.x = float(x)
            msg.orientation.y = float(y)
            msg.orientation.z = float(z)
            msg.orientation.w = float(w)
            self.pose_pub.publish(msg)

            # Publish surge, sway, and heave force setpoints
            # in range [-1, 1] as a Twist msg
            msg = Twist()
            msg.linear.x = float(self.drone.motion.surge)
            msg.linear.y = float(self.drone.motion.sway)
            msg.linear.z = float(self.drone.motion.heave)
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            # yaw rate moment setpoint in range [-1, 1]
            msg.angular.z = float(self.drone.motion.yaw)
            self.thruster_force_norm_pub.publish(msg)

            # Publishing light level as an Int32 msg
            msg = Int32()
            msg.data = int(self.drone.lights)
            self.lights_lvl_pub.publish(msg)

            # Publish battery percentage in [0, 100] range as Int32
            msg = Int32()
            msg.data = int(self.drone.battery_state_of_charge)
            self.battery_percentage_pub.publish(msg)

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
            msg.bitrate = self.drone.camera.bitrate
            msg.exposure = self.drone.camera.exposure
            msg.frames_per_second = self.drone.camera.framerate
            msg.is_recording = self.drone.camera.is_recording
            msg.hue = self.drone.camera.hue
            msg.recording_time = self.drone.camera.record_time
            msg.resolution = self.drone.camera.resolution
            msg.tilt_angle = self.drone.camera.tilt.angle
            msg.tilt_speed_ref = 0
            msg.whitebalance = self.drone.camera.whitebalance
            self.camera_params_pub.publish(msg)

            # Publishing boost gain
            msg = Float32()
            msg.data = float(self.drone.motion.boost)
            self.boost_gain_pub.publish(msg)

            # Publishing slow gain
            msg = Float32()
            msg.data = float(self.drone.motion.slow)
            self.slow_gain_pub.publish(msg)

            # Publishing water density
            msg = Int32()
            msg.data = int(self.drone.config.water_density)
            self.water_density_pub.publish(msg)

            # Publish ROV's IP address
            msg = String()
            msg.data = self.drone.logs.ip
            self.ip_address_pub.publish(msg)

            # Publish ROV's software version
            msg = String()
            msg.data = self.drone.software_version
            self.software_version_pub.publish(msg)

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
            Twist, "thruster_force_norm_ref", self.thruster_force_norm_ref_callback, 10)
        self.create_subscription(
            Int32, "lights_lvl_ref", self.lights_lvl_ref_callback, 10)
        self.create_subscription(
            Int32, "auto_mode_ref", self.auto_mode_ref_callback, 10)
        self.create_subscription(
            BlueyeCameraParams, "camera_params_ref", self.camera_params_ref_callback, 10)
        self.create_subscription(
            Float32, "boost_gain_ref", self.boost_gain_ref_callback, 10)
        self.create_subscription(
            Float32, "slow_gain_ref", self.slow_gain_ref_callback, 10)
        self.create_subscription(
            Int32, "water_density_ref", self.water_density_ref_callback, 10)

    def initialize_publishers(self):
        print("Initializing ROS publishers")
        # Initialize ROS publishers of ROV variables - ROV get topics
        self.pose_pub = self.create_publisher(Pose, "pose", 10)
        self.thruster_force_norm_pub = self.create_publisher(
            Twist, "thruster_force_norm", 10)
        self.lights_lvl_pub = self.create_publisher(
            Int32, "lights_lvl", 10)
        self.connected_status_pub = self.create_publisher(
            Bool, "connected_status", 10)
        self.auto_mode_pub = self.create_publisher(
            Int32, "auto_mode", 10)  # 00, 01, 10, 11 active state for depth-heading
        self.camera_params_pub = self.create_publisher(
            BlueyeCameraParams, "camera_params", 10)
        self.battery_percentage_pub = self.create_publisher(
            Int32, "battery_percentage", 10)
        self.boost_gain_pub = self.create_publisher(
            Float32, "boost_gain", 10)
        self.slow_gain_pub = self.create_publisher(
            Float32, "slow_gain", 10)
        self.water_density_pub = self.create_publisher(
            Int32, "water_density", 10)
        self.ip_address_pub = self.create_publisher(
            String, "ip_address", 10)
        self.software_version_pub = self.create_publisher(
            String, "software_version", 10)

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
