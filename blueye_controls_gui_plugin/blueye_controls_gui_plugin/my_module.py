import rclpy
#import rospkg
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from blueye_ros2_msgs.msg import BlueyeCameraParams

import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
#from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import *


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        #ui_file = os.path.join(rospkg.RosPack().get_path('blueye_controls_gui_plugin'), 'resource', 'blueye_param_dashboard_gui.ui')

        ui_file = os.path.join(
            get_package_share_directory('blueye_controls_gui_plugin'),
            'resource',
            'blueye_param_dashboard_gui.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Start ROS2 node, publishers and subscribers
        # super().__init__('blueye_controls_gui_plugin')
        # rclpy.init()
        self.node = rclpy.create_node('blueye_controls_gui_node')
        self.initialize_publishers()
        self.initialize_subscribers()

        # Set all controls from GUI to their default values
        self.battery_lvl = 50
        self.auto_depth_mode = 0
        self.auto_heading_mode = 0
        
        self.exposure_auto = True
        self.whitebalance_auto = True
        self.bitrate = 8000000
        self.exposure = -1
        self.framerate = 25
        self.is_recording = False
        self.hue = 0
        self.recording_time = -1
        self.resolution = 1080
        self.tilt_angle = 0.0
        self.tilt_speed_ref = 0
        self.whitebalance = -1

        self.lights = 0
        self.water_density = 1025
        self.boost = 0.5
        
        
        

        self.publish_camera_params_ref()
        self.publish_lights_ref()
        self.publish_boost_ref()
        self.publish_water_density_ref()
        self.publish_auto_mode_ref()        
        
        # Find auto depth and heading modes QPushBUttons and connect 
        #their Clicked signal with callbacks   
        self.autoDepthModeButton = self._widget.findChild(QPushButton, 'autoDepthModeButton')
        self.autoDepthModeButton.clicked.connect(self.autoDepthModeButtonClicked)

        self.autoHeadingModeButton = self._widget.findChild(QPushButton, 'autoHeadingModeButton')
        self.autoHeadingModeButton.clicked.connect(self.autoHeadingModeButtonClicked)

        # Find exposure_feature-related blocks and connect their signals with callbacks
        self.exposureCheckBox = self._widget.findChild(
            QCheckBox, 'exposureEnableCheckBox')
        self.exposureCheckBox.stateChanged.connect(
            self.exposureCheckBoxStateChanged)

        self.exposureSliderValueLabel = self._widget.findChild(
            QLabel, 'exposureSliderValueLabel')

        self.exposureSlider = self._widget.findChild(
            QSlider, 'exposureSlider')
        self.exposureSlider.valueChanged.connect(
            self.exposureSliderValueChanged)

        # Find white_balance_feature-related blocks and connect their signals with callbacks
        self.whiteBalanceCheckBox = self._widget.findChild(
            QCheckBox, 'whiteBalanceEnableCheckBox')
        self.whiteBalanceCheckBox.stateChanged.connect(
            self.whiteBalanceCheckBoxStateChanged)

        self.whiteBalanceSliderValueLabel = self._widget.findChild(
            QLabel, 'whiteBalanceSliderValueLabel')

        self.whiteBalanceSlider = self._widget.findChild(
            QSlider, 'whiteBalanceSlider')
        self.whiteBalanceSlider.valueChanged.connect(
            self.whiteBalanceSliderValueChanged)

        # Find hue_feature-related blocks and connect their signals with callbacks
        self.hueSliderValueLabel = self._widget.findChild(
            QLabel, 'hueSliderValueLabel')

        self.hueSlider = self._widget.findChild(
            QSlider, 'hueSlider')
        self.hueSlider.valueChanged.connect(
            self.hueSliderValueChanged)

        # Find bitrate_feature-related blocks and connect their signals with callbacks
        self.bitrateSliderValueLabel = self._widget.findChild(
            QLabel, 'bitrateSliderValueLabel')

        self.bitrateSlider = self._widget.findChild(
            QSlider, 'bitrateSlider')
        self.bitrateSlider.valueChanged.connect(
            self.bitrateSliderValueChanged)

        # Find framerate_feature-related blocks and connect their signals with callbacks
        self.framerateComboBox = self._widget.findChild(
            QComboBox, 'framerateComboBox')
        self.framerateComboBox.currentTextChanged.connect(
            self.framerateComboBoxCurrentTextChanged)

        # Find resolution_feature-related blocks and connect their signals with callbacks
        self.resolutionComboBox = self._widget.findChild(
            QComboBox, 'resolutionComboBox')
        self.resolutionComboBox.currentTextChanged.connect(
            self.resolutionComboBoxCurrentTextChanged)

        # Find lights_feature-related blocks and connect their signals with callbacks
        self.lightsSliderValueLabel = self._widget.findChild(
            QLabel, 'lightsSliderValueLabel')

        self.lightsSlider = self._widget.findChild(
            QSlider, 'lightsSlider')
        self.lightsSlider.valueChanged.connect(
            self.lightsSliderValueChanged)

        # Find water_density_feature-related blocks and connect their signals with callbacks
        self.waterDensityComboBox = self._widget.findChild(
            QComboBox, 'waterDensityComboBox')
        self.waterDensityComboBox.currentTextChanged.connect(
            self.waterDensityComboBoxCurrentTextChanged)

        # Find boost_feature-related blocks and connect their signals with callbacks
        self.boostSliderValueLabel = self._widget.findChild(
            QLabel, 'boostSliderValueLabel')

        self.boostSlider = self._widget.findChild(
            QSlider, 'boostSlider')
        self.boostSlider.valueChanged.connect(
            self.boostSliderValueChanged)

    def initialize_subscribers(self):
        print("Initializing ROS subscribers")
        """# Initialize ROS subscribers to ROV variables' reference - ROV set topics
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
        """

        self.node.create_subscription(Int32, "battery_percentage",
                                 self.battery_percentage_callback, 10) 

        #self.node.create_subscription(
        #    BlueyeCameraParams, "camera_params", self.camera_params_callback, 10)

    def camera_params_callback(self, msg):
        """msg = BlueyeCameraParams()
        self.bitrate = int(msg.bitrate/1000000) 
        self.exposure_= msg.exposure
        self.framerate = msg.frames_per_second 
        self.is_recording = msg.is_recording
        self.hue = msg.hue
        self.recording_time = msg.recording_time 
        self.resolution = msg.resolution
        self.tilt_angle = msg.tilt_angle
        self.tilt_speed_ref = msg.tilt_speed_ref
        self.whitebalance = msg.whitebalance"""
        return

    def battery_percentage_callback(self, msg):
        self.battery_lvl = msg.data
        print(str(msg.data))
        string = str(self.battery_lvl) + '%'
        self.batteryLevelLabel.setText(string)

    def initialize_publishers(self):
        print("Initializing ROS publishers")
        """# Initialize ROS publishers of ROV variables - ROV get topics
        #self.pose_pub = self.create_publisher(Pose, "pose", 10)
        # self.thruster_force_norm_pub = self.create_publisher(
        #   Twist, "thruster_force_norm", 10)
        self.lights_lvl_pub = self.create_publisher(
            Int32, "lights_lvl_ref", 10)
        self.connected_status_pub = self.create_publisher(
            Bool, "connected_status", 10)
        self.auto_mode_pub = self.create_publisher(
            Int32, "auto_mode_ref", 10)  # 00, 01, 10, 11 active state for depth-heading
        
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
            String, "software_version", 10) """

        self.camera_params_pub = self.node.create_publisher(
            BlueyeCameraParams, "camera_params_ref", 10)

        self.lights_pub = self.node.create_publisher(
            Int32, "lights_lvl_ref", 10)

        self.boost_pub = self.node.create_publisher(
            Float32, "boost_gain_ref", 10)
            
        self.water_density_pub = self.node.create_publisher(
            Int32, "water_density_ref", 10)
        
        self.auto_mode_pub = self.node.create_publisher(
            Int32, "auto_mode_ref", 10)
            
    def publish_auto_mode_ref(self):
        msg = Int32()
        msg.data = 2*self.auto_depth_mode + self.auto_heading_mode
        self.auto_mode_pub.publish(msg)
        return

    def publish_camera_params_ref(self):
        msg = BlueyeCameraParams()
        msg.bitrate = self.bitrate
        msg.exposure = self.exposure
        msg.frames_per_second = self.framerate
        msg.is_recording = self.is_recording
        msg.hue = self.hue
        msg.recording_time = self.recording_time
        msg.resolution = self.resolution
        msg.tilt_angle = self.tilt_angle
        msg.tilt_speed_ref = self.tilt_speed_ref
        msg.whitebalance = self.whitebalance
        self.camera_params_pub.publish(msg)

    def publish_lights_ref(self):
        msg = Int32()
        msg.data = self.lights
        self.lights_pub.publish(msg)

    def publish_water_density_ref(self):
        msg = Int32()
        msg.data = self.water_density
        self.water_density_pub.publish(msg)

    def publish_boost_ref(self):
        msg = Float32()
        msg.data = self.boost
        self.boost_pub.publish(msg)

    def autoDepthModeButtonClicked(self):
        if self.auto_depth_mode == 0:
            self.auto_depth_mode = 1
        elif self.auto_depth_mode == 1:
            self.auto_depth_mode = 0
        self.publish_auto_mode_ref()
        
    def autoHeadingModeButtonClicked(self):
        if self.auto_heading_mode == 0:
            self.auto_heading_mode = 1
        elif self.auto_heading_mode == 1:
            self.auto_heading_mode = 0
        self.publish_auto_mode_ref()

    def exposureCheckBoxStateChanged(self):
        self.exposure_auto = (not self.exposureCheckBox.checkState())
        if self.exposure_auto:
            self.exposure = -1
            self.exposureSliderValueLabel.setText("Auto")
            self.exposureSlider.setEnabled(False)

        else:
            self.exposureSlider.setEnabled(True)
            self.exposure = self.exposureSlider.value()
            string = str(self.exposure) + '/1000 s'
            self.exposureSliderValueLabel.setText(string)

        self.publish_camera_params_ref()

    def exposureSliderValueChanged(self):
        if not self.exposure_auto:
            self.exposure = self.exposureSlider.value()
            string = str(self.exposure) + '/1000 s'
            self.exposureSliderValueLabel.setText(string)
            self.publish_camera_params_ref()

    def whiteBalanceCheckBoxStateChanged(self):
        self.whitebalance_auto = (not self.whiteBalanceCheckBox.checkState())
        print("WB auto " + str(self.whitebalance_auto))
        if self.whitebalance_auto:
            self.whitebalance = -1
            self.whiteBalanceSliderValueLabel.setText("Auto")
            self.whiteBalanceSlider.setEnabled(False)
            self.publish_camera_params_ref()
        else:
            self.whiteBalanceSlider.setEnabled(True)
            self.whitebalance = self.whiteBalanceSlider.value()
            string = str(self.whitebalance) + ' K'
            self.whiteBalanceSliderValueLabel.setText(string)
            self.publish_camera_params_ref()

    def whiteBalanceSliderValueChanged(self):
        if not self.whitebalance_auto:
            self.whitebalance = self.whiteBalanceSlider.value()
            string = str(self.whitebalance) + ' K'
            self.whiteBalanceSliderValueLabel.setText(string)
            self.publish_camera_params_ref()

    def hueSliderValueChanged(self):
        self.hue = self.hueSlider.value()
        string = str(self.hue)
        self.hueSliderValueLabel.setText(string)
        self.publish_camera_params_ref()

    def bitrateSliderValueChanged(self):
        self.bitrate = self.bitrateSlider.value()*1000000
        string = str(self.bitrate/1000000) + ' Mbps'
        self.bitrateSliderValueLabel.setText(string)
        self.publish_camera_params_ref()

    def framerateComboBoxCurrentTextChanged(self):
        chosen_option = self.framerateComboBox.currentText()
        if chosen_option == '25 fps':
            self.framerate = 25
        elif chosen_option == '30 fps':
            self.framerate = 30
        self.publish_camera_params_ref()
    
    def resolutionComboBoxCurrentTextChanged(self):
        chosen_option = self.resolutionComboBox.currentText()
        if chosen_option == '720p':
            self.resolution = 720
        elif chosen_option == '1080p':
            self.resolution = 1080
        self.publish_camera_params_ref()
    
    def waterDensityComboBoxCurrentTextChanged(self):
        chosen_option = self.waterDensityComboBox.currentText()
        print(str(chosen_option))
        if chosen_option == "Salty - 1025 g/l":
            self.water_density = 1025
        elif chosen_option == "Freshwater - 997g/l":
            self.water_density = 997
        elif chosen_option == "Brackish - 1011 g/l":
            self.water_density = 1011
        else:
            self.water_density = int(round(float(chosen_option)))
        self.publish_water_density_ref()

    def lightsSliderValueChanged(self):
        self.lights = self.lightsSlider.value()
        string = str(round(self.lights/255*100, 2)) + '%'
        self.lightsSliderValueLabel.setText(string)
        self.publish_lights_ref()

    def boostSliderValueChanged(self):
        self.boost = self.boostSlider.value()/100.0
        string = str(round(self.boost*100, 2)) + '%'
        self.boostSliderValueLabel.setText(string)
        self.publish_boost_ref()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
