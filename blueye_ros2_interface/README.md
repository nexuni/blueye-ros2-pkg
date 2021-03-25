# blueye_ros2_interface
ROS2 node for interfacing with Blueye SDK.
 
### Dependences installation

* cv_bridge  ROS Foxy pkg
* image_transport ROS Foxy pkg
* OpenCV4 
* gstreamer-related libs
* video_stream_opencv ROS Foxy pkg

### ROS2 Foxy running Python 3 assumed
### Otherwise change ros2-distro in the following commands
sudo apt update &&
sudo apt upgrade &&
sudo apt install ros-foxy-vision-opencv &&
sudo apt install ros-foxy-image-transport &&
sudo apt install libopencv-dev python3-opencv &&
python3 -c "import cv2; print(cv2.__version__); exit()" &&
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc \
gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa &&

### Updating ros_shared and gscam submodules and building 
cd <ros2_workspace>/src/blueye-ros2-pkg 
git submodule init 
git submodule update 
cd ~/<ros2_workspace>/
colcon build


### Sourcing ROS Foxy and its workspace 
## Topside
echo $ROS_DISTRO
source /opt/ros/foxy/setup.bash
source <path_to_ros2_workspace>/install/setup.bash
# If you're running only one ROS version, it is convinient to 
# add it directly in the ~/.bashrc
sudo echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
sudo echo "source <path_to_ros_workspace>/install/setup.bash" >> ~/.bashrc
# Otherwise sourcing ROS2 and its workspace is neccesary to run in each terminal 
# in which you plan to use ROS commands and run/launch ROS stuff

## Bottom-side
ssh <bttm_side_user>@<bttm_side_ip_address> 
# and repeat the same sourcing similarly as for the topside


### Remote access over network in ROS12
## Adding hostnames on the top- and bottom-side 
## It is not necessary in ROS2, but is convinient for pinging by hostname etc.
# Top-side 
echo $HOSTNAME 
sudo echo "<bttm_side_ip_address> <bttm_side_hostname>" >> /etc/hosts
ping <top_side_hostname>
ping <bttm_side_hostname>

# Bottom-side
ssh <bttm_side_user>@<bttm_side_ip_address>
echo $HOSTNAME 
sudo echo "<top_side_ip_address> <top_side_hostname>" >> /etc/hosts
ping <top_side_hostname>
ping <bttm_side_hostname>


### Adding a static IP address in order to access the ROV
## This applies if you're connecting the ROV to the bottom-side computer (e.g. on an ASV) over ethernet 
## switch and you have static IP addresses for all devices in your LAN

# Set your IP address in 192.168.1.1/24 range except 192.168.1.101
# Blueye ROV is usually at IP addr 192.168.1.101/24 - contact Blueye Robotics if otherwise
# Search for all devices on this subnet
nmap -sP 192.168.1.1-255
# Your hostaname and at least one other device should appear. 
ping 192.168.1.101 
# or any new device whose IP isn't known to you beforehand



# Look for the ethernet interface name by using the ifconfig command
ifonfig 
sudo ip addr add 192.168.1.101/24 dev <ether-interface-name>
# It would be convinient to add this command to .bashrc so it is 



### Usage 
 
## Direct connection from the operator to the camera
## on the topside w/o bottom-side - runs both the RTSP stream
## node and the GUI
ros2 launch blueye_ros2_interface start_blueye_interface.launch.py 
# In rqt_image_view GUI switch to /blueye/camera_img_raw topic


## Connecting the IP cam into ROS2 @ bottom-side (e.g. an ASV to which the ROV is connected by tether)
# Bottom-side (used for online processing and logging)
ssh <bttm_side_user>@<bttm_side_ip_address>
ros2 launch blueye_ros2_interface start_blueye_interface_bttm_side.launch.py 

# Topside (used for surveilance and PTZ control by the operator)
ros2 launch blueye_ros2_interface start_blueye_interface_top_side_with_gscam_node.launch.py 
# DO NOT switch to /blueye/camera_img_raw topic in rqt_image_view GUI, 
# especially if you're connecting to the bottom-side over Wi-Fi
# since it will not get images from the bottom side that publishes ROV's cam
# and gscam node at the bottom side will leak memory and eventually die.
# ROS2 does enable automatic ROS node and topic discovery, but sending Image msgs over
# WiFi does not work (still).


