#!/usr/bin/env python3

import socket
import struct
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter

import time

class OculusSonar(Node):
    """
    Class to handle Sonar operations.
    """
    # Constant ports for Oculus comms
    TCP_PORT = 52100

    def __init__(self, node_name='oculus_sonar'):
        """
        Initialize the node.
        """
        super().__init__(node_name)

        # Parameters
        self.declare_parameter("tcp_ip", "192.168.115.42")
        self.tcp_ip = self.get_parameter("tcp_ip").get_parameter_value().string_value

        # Dynamic reconfigure client
        self.client = self.create_client(SetParameters, '/dynamic_reconfigure/set_parameters')

        self.config_change = False

        self.fire_message = self.build_simplefire_msg()

        # Sonar image publisher
        self.image_pub = self.create_publisher(Image, '/sonar_image_raw', 1)
        self.azimuth = 0
        self.ranges = 0

        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_tcp()

    def connect_tcp(self):
        """
        Connect to the TCP server.
        """
        self.tcp_sock.connect((self.tcp_ip, self.TCP_PORT))

    def send_tcp_msg(self, msg):
        """
        Send TCP message to the connected port.
        """
        self.tcp_sock.sendto(msg, (self.tcp_ip, self.TCP_PORT))

    def recv_tcp_msg(self):
        """
        Receive TCP message from the connected port.
        """
        data = self.tcp_sock.recv(1500)
        return data

    def build_simplefire_msg(self,
                             masterMode=1, pingRate=0,
                             gamma=150, gain=0, range_m=10,
                             vos=0, salinity=0):
        """
        Build simple fire message.
        """
        oculusId = b'\x53\x4f'
        srcDeviceId = b'\x00\x00'
        dstDeviceId = b'\x00\x00'
        msgId = b'\x15\x00'
        msgVersion = b'\x00\x00'
        payloadSize = b'\x00\x00\x00\x00'
        spare2 = b'\x00\x00'
        fire_message = oculusId + srcDeviceId + dstDeviceId + msgId + msgVersion + payloadSize + spare2

        if not self.config_change:
            fire_message += struct.pack('B', masterMode)
            fire_message += struct.pack('B', pingRate)
            networkSpeed = b'\xff'
            fire_message += networkSpeed
            fire_message += struct.pack('B', gamma)
            flags = b'\x19'
            fire_message += flags
            fire_message += struct.pack('d', range_m)
            fire_message += struct.pack('d', gain)
            fire_message += struct.pack('d', vos)
            fire_message += struct.pack('d', salinity)
        else:
            fire_message += struct.pack('B', masterMode)
            fire_message += struct.pack('B', pingRate)
            networkSpeed = b'\xff'
            fire_message += networkSpeed
            fire_message += struct.pack('B', gamma)
            flags = b'\x19'
            fire_message += flags
            fire_message += struct.pack('d', range_m)
            fire_message += struct.pack('d', gain)
            fire_message += struct.pack('d', vos)
            fire_message += struct.pack('d', salinity)

        self.config_change = False

        return fire_message

    def process_n_publish(self, data):
        """
        Process the received sonar message and
        publish the sonar image on a ROS Image
        message.
        """
        # Get no.bearings and no.ranges
        dim = struct.unpack('HH', data[106:110])
        # Starting offset for sonar image
        img_offset = struct.unpack('I', data[110:114])[0]
        # Build greyscale image from echo intensity data
        img = np.frombuffer(data[img_offset:], dtype='uint8')
        img_temp_buffer = img.copy()
        for i in range(dim[0]):
            img_temp_buffer[i*dim[1] : (i+1)*dim[1]] = img[(dim[0] - i - 1)*dim[1] : (dim[0] - i)*dim[1]]
        img_size = struct.unpack('I', data[114:118])[0]
        msg_len = struct.unpack('I',data[10:14])[0]
        
        if dim[0]*dim[1] != img_size:
            self.get_logger().warn("Message dims {0} don't match ping result info {1}. Dropping frame.".format(dim, len(img)))
        else:            
            #print(dim)
            master_mode = data[16]
            if master_mode == 1:
                self.azimuth = 130
            else:
                self.azimuth = 80
            self.ranges = struct.unpack('d', data[21:29])[0]

            # ros publish sonar image
            image_temp = Image()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "["+ str(float(self.azimuth)) + " " + str(float(self.ranges)) + "]"
            image_temp.height = dim[0]
            image_temp.width = dim[1]
            image_temp.encoding = 'mono8'
            image_temp.data = np.array(img_temp_buffer).tobytes()
            image_temp.header = header
            image_temp.step = dim[1]
            self.image_pub.publish(image_temp)

def main(args=None):
    """
    Main method for the ROS2 node.
    """
    rclpy.init(args=args)
    sonar = OculusSonar()

    while rclpy.ok():
        rclpy.spin_once(sonar, timeout_sec=0.1)

        if sonar.config_change:
            sonar.fire_message = sonar.build_simplefire_msg()
        
        sonar.send_tcp_msg(sonar.fire_message)
        data = sonar.recv_tcp_msg()

        if data is None:
            continue
        if len(data) < 100:
            continue

        msg_len = struct.unpack('I',data[10:14])[0]
        while len(data) < msg_len:
            data += sonar.recv_tcp_msg()

        sonar.process_n_publish(data)

    sonar.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()