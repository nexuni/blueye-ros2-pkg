import rclpy
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
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QLabel, QPushButton, \
    QComboBox, QSlider, QCheckBox, QProgressBar

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

        # Set all controls from GUI to their default values
        # Initialize all MyPlugin class properties
        self.initialize_class_properties()

        # Start ROS2 node, publishers and subscribers
        # super().__init__('blueye_controls_gui_plugin')
        # rclpy.init()
        self.node = context.node
        self.initialize_ros_stuff()

        # Instantiate UI elements from the loaded .ui file
        self.instantiate_ui_elements()
        
        # Loading class properties values from ROV during start up
        self.update_class_properties()
        
        #Testing somethin
        
        

    def initialize_class_properties(self):
        self.battery_lvl = 50
        self.auto_depth_mode = 0
        self.auto_heading_mode = 0
        self.manual_control_enable = False

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
        self.boost = 0.0
        
        # Exposure slider values placed in lists
        self.exposureValueOutputWriteList = ["1/1000", "1/800", "1/640", "1/500", "1/400", "1/320", "1/250", "1/200", "1/160", "1/125", "1/100", "1/80", "1/60", "1/50", "1/40", "1/30", "1/20", "1/15", "1/10", "1/8", "1/6", "1/5", "1/4", "0.3", "0.4", "0.5", "0.6", "0.8", "1", "1.3", "1.6", "2", "2.5", "3.2", "4", "5"]
        self.exposureValueList = [0.001, 0.0012, 0.0015, 0.002, 0.0025, 0.0031, 0.004, 0.005, 0.0062, 0.008, 0.01, 0.0125, 0.0167, 0.02, 0.025, 0.0333, 0.05, 0.0667, 0.1, 0.125, 0.1667, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.8, 1, 1.3, 1.6, 2, 2.5, 3.2, 4, 5]
        
    def update_class_properties(self):
        #initializing gui from ROV
        #creating wait time for cca 100 ms
        time.sleep(0.1)
        
        try:
           # self.camera_params_callback
           # self.lights_callback
           # self.water_density_callback
           # self.boost_callback
            
            self.battery_lvl = 50
            self.auto_depth_mode = 0
            self.auto_heading_mode = 0
            self.manual_control_enable = False

            self.exposure_auto = True
            self.whitebalance_auto = True
            bitrateTemp = int(self.bitrate/1000000)
            if self.exposure == -1:
                exposureTemp = 0
            else:
                exposureTemp = int(self.exposureValueList.index(self.exposure/1000)+1)
            if self.framerate == 25:
                framerateTemp = 0
            else:
                framerateTemp = 1
            is_recordingTemp = int(self.is_recording)
            hueTemp = int(self.hue/4)
            recording_timeTemp = int(self.recording_time)
            if self.resolution == 1080:
                resolutionTemp = 0
            else:
                resolutionTemp = 1
            tilt_angleTemp = int(self.tilt_angle)
            tilt_speed_refTemp = int(self.tilt_speed_ref)
            if self.whitebalance == -1:
                whitebalanceTemp = 0
            else:
                whitebalanceTemp = int((self.whitebalance-2800)/500)
            
            
            
            lightsTemp = int(round(self.lights*10/255))
            if self.water_density == 1025:
                water_densityTemp = 0
            elif self.water_density == 997:
                water_densityTemp = 1
            else:
                water_densityTemp = 2
            boostTemp = int(self.boost*10)
            print("Updating values from ROV - Succeeded")
        except:
            

            self.battery_lvl = 50
            self.auto_depth_mode = 0
            self.auto_heading_mode = 0
            self.manual_control_enable = False

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
            self.boost = 0.0

            print("Updating values from ROV - Failed")
        
        self.autoDepthModeButton.setChecked(False)
        self.autoHeadingModeButton.setChecked(False)
        self.manualAutoPushButton.setChecked(False)

        self.exposureCheckBox.setChecked(False)
        self.whiteBalanceCheckBox.setChecked(False) #treba napraviti i za njih da čitaju

        self.exposureSlider.setValue(exposureTemp)
        self.whiteBalanceSlider.setValue(whitebalanceTemp)
        self.hueSlider.setValue(hueTemp)
        self.bitrateSlider.setValue(bitrateTemp)
        self.framerateComboBox.setCurrentIndex(framerateTemp)
        self.resolutionComboBox.setCurrentIndex(resolutionTemp)
        self.lightsSlider.setValue(lightsTemp)
        self.waterDensityComboBox.setCurrentIndex(water_densityTemp)
        self.boostSlider.setValue(boostTemp)

    def instantiate_ui_elements(self):
        # Find battery level QLabel
        self.batteryLevelLabel = self._widget.findChild(
            QLabel, 'batteryLevelLabel')
        self.batteryPercentageBar = self._widget.findChild(
            QProgressBar, 'batteryPercentageBar')
        

        # Find auto depth and heading modes QPushButtons and connect
        # their Clicked signal with callbacks
        self.autoDepthModeButton = self._widget.findChild(
            QPushButton, 'autoDepthModeButton')
        self.autoDepthModeButton.clicked.connect(
            self.autoDepthModeButtonClicked)

        self.autoHeadingModeButton = self._widget.findChild(
            QPushButton, 'autoHeadingModeButton')
        self.autoHeadingModeButton.clicked.connect(
            self.autoHeadingModeButtonClicked)

        
        #Find manual mode QPushButton and connect their Clicked signal with callbacks
        
        self.manualAutoPushButton = self._widget.findChild(
            QPushButton, 'manualAutoPushButton')
        self.manualAutoPushButton.clicked.connect(
            self.manualAutoPushButtonClicked)


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
        

        self.boostSliderRealValueLabel = self._widget.findChild(
            QLabel, 'boostSliderRealValueLabel')
        self.waterDensityDropMenuRealValueLabel = self._widget.findChild(
            QLabel, 'waterDensityDropMenuRealValueLabel')
        self.lightsSliderRealValueLabel = self._widget.findChild(
            QLabel, 'lightsSliderRealValueLabel')
        self.resolutionComboBoxRealValueLabel = self._widget.findChild(
            QLabel, 'resolutionComboBoxRealValueLabel')
        self.bitrateSliderRealValueLabel = self._widget.findChild(
            QLabel, 'bitrateSliderRealValueLabel')
        self.hueSliderRealValueLabel = self._widget.findChild(
            QLabel, 'hueSliderRealValueLabel')
        self.framerateComboBoxRealValueLabel = self._widget.findChild(
            QLabel, 'framerateComboBoxRealValueLabel')
        self.whiteBalanceSliderRealValueLabel = self._widget.findChild(
            QLabel, 'whiteBalanceSliderRealValueLabel')
        self.exposureSliderRealValueLabel = self._widget.findChild(
            QLabel, 'exposureSliderRealValueLabel')
            

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
    
    def manualAutoPushButtonClicked(self):
        if self.manual_control_enable == False:
            self.manual_control_enable = True
        elif self.manual_control_enable == True:
            self.manual_control_enable = False
        self.publish_manual_control_enable_ref()

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
            self.exposure = int(self.exposureValueList[self.exposureSlider.value()-1]*1000)
            string = self.exposureValueOutputWriteList[self.exposureSlider.value()-1]+ ' s'
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
            self.whitebalance = 2800+self.whiteBalanceSlider.value()*500
            string = str(self.whitebalance) + ' K'
            self.whiteBalanceSliderValueLabel.setText(string)
            self.publish_camera_params_ref()

    def whiteBalanceSliderValueChanged(self):
        if not self.whitebalance_auto:
            self.whitebalance = 2800+self.whiteBalanceSlider.value()*500
            string = str(self.whitebalance) + ' K'
            self.whiteBalanceSliderValueLabel.setText(string)
            self.publish_camera_params_ref()

    def hueSliderValueChanged(self):
        self.hue = self.hueSlider.value()*4
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
        self.lights = round(self.lightsSlider.value()*(255/10))
        string = str(round(self.lights/255*100)) + '.0%'
        self.lightsSliderValueLabel.setText(string)
        self.publish_lights_ref()
        

    def boostSliderValueChanged(self):
        self.boost = self.boostSlider.value()/10.0
        string = str(round(self.boost*100, 2)) + '%'
        self.boostSliderValueLabel.setText(string)
        self.publish_boost_ref()
        """
        valueFromROV = 0.2
        self.sliderRealValueCheck(0.2, self.boost, self.boostSliderRealValueLabel)"""

        
    def sliderRealValueCheck(self, valueFromROV, valueFromGUI, sliderCheck):
        #parameters: self, what to compare first, what to compare second, slider to change
        #valueFromROV = 0.2 #očitati s vozila vrijednost
        print("ROV: "+str(valueFromROV))
        print("GUI: "+str(valueFromGUI))
        if valueFromROV == valueFromGUI:
            text = "✓"
            color = "<span style=\" font-size:11pt; font-weight:600; color:#3BB143;\" >"+text+"</span>"
            sliderCheck.setText(text)
        elif valueFromROV < valueFromGUI:
            text = "↑"
            color = "<span style=\" font-size:11pt; font-weight:600; color:#E76E00;\" >"+text+"</span>"
            sliderCheck.setText(text)
        else:
            text = "↓"
            color = "<span style=\" font-size:11pt; font-weight:600; color:#E76E00;\" >"+text+"</span>"
            sliderCheck.setText(text)

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

    def initialize_ros_stuff(self):
        #self.node = rclpy.create_node('blueye_controls_gui_node')
        self.RATE = 10  # Hz
        self.initialize_publishers()
        self.initialize_subscribers()
        # self.initialize_timer()  # ???
        self.publish_camera_params_ref()
        self.publish_lights_ref()
        self.publish_boost_ref()
        self.publish_water_density_ref()
        self.publish_auto_mode_ref()
        self.publish_manual_control_enable_ref()

    def initialize_subscribers(self):
        print("Initializing ROS subscribers")
        """
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
        """
        

        self.node.create_subscription(
            Int32, "lights_lvl", self.lights_lvl_callback, 10)
        self.node.create_subscription(
            Int32, "auto_mode", self.auto_mode_callback, 10)
        self.node.create_subscription(
            BlueyeCameraParams, "camera_params", self.camera_params_callback, 10)
        """self.node.create_subscription(
            BlueyeCameraParams, "camera_params_ref", self.camera_params_callback, 10)
        self.node.create_subscription(
            BlueyeCameraParams, "lights_lvl_ref", self.lights_callback, 10)
        self.node.create_subscription(
            BlueyeCameraParams, "boost_gain_ref", self.boost_callback, 10)
        self.node.create_subscription(
            BlueyeCameraParams, "water_density_ref", self.water_density_callback, 10)"""
        self.node.create_subscription(
            Float32, "boost_gain", self.boost_gain_callback, 10)
        self.node.create_subscription(
            Int32, "water_density", self.water_density_callback, 10)
        

        self.node.create_subscription(Int32, "battery_percentage",
                                      self.battery_percentage_callback, 10)

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

        #Manual control publisher
        self.manual_control_enable_pub = self.node.create_publisher(
            Bool, "manual_control_enable_ref", 10)



    def camera_params_callback(self, msg):
        # Callback for camera parametars and where it checks and compares for real value
        self.sliderRealValueCheck(int(msg.bitrate), self.bitrate, self.bitrateSliderRealValueLabel)
        self.sliderRealValueCheck(msg.exposure, self.exposure, self.exposureSliderRealValueLabel)
        self.sliderRealValueCheck(msg.frames_per_second, self.framerate, self.framerateComboBoxRealValueLabel)
        self.sliderRealValueCheck(msg.hue, self.hue, self.hueSliderRealValueLabel)
        self.sliderRealValueCheck(msg.resolution, self.resolution, self.resolutionComboBoxRealValueLabel)
        self.sliderRealValueCheck(msg.whitebalance, self.whitebalance, self.whiteBalanceSliderRealValueLabel)
        return

    def auto_mode_callback(self, msg):
        #        #Callback for auto mode
        return


    def lights_lvl_callback(self, msg):
        self.sliderRealValueCheck(msg.data, self.lights, self.lightsSliderRealValueLabel)


    def battery_percentage_callback(self, msg):
        self.battery_lvl = msg.data
        string = str(self.battery_lvl) + '%'
        self.batteryPercentageBar.setValue(self.battery_lvl)

        #self.batteryLevelLabel.setText(string)
        return

    def boost_gain_callback(self, msg):
        #Callback for boost gain where it checks and compares for real value
        self.sliderRealValueCheck(msg.data, self.boost, self.boostSliderRealValueLabel)
        return


    def water_density_callback(self, msg):
        #Callback for water density where it checks and compares for real value
        print(str(msg.data))
        print(str(self.water_density))
        self.sliderRealValueCheck(msg.data, self.water_density, self.waterDensityDropMenuRealValueLabel)
        return


    def publish_auto_mode_ref(self):
        msg = Int32()
        msg.data = 2*self.auto_depth_mode + self.auto_heading_mode
        self.auto_mode_pub.publish(msg)
        return
        
    def publish_manual_control_enable_ref(self):
        #TODO Ispraviti i dovršiti ovo
        print('Manual control: '+str(self.manual_control_enable))
        msg = Bool()
        msg.data = self.manual_control_enable
        self.manual_control_enable_pub.publish(msg)
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

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
