#!/usr/bin/env python3

import rospy

from blueye.sdk import Pioneer

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class blueye_interface():
	def control_information_callback(self, data):
		surge_val = 0.25 * data.linear.x
		sway_val = 0.25 * data.linear.y
		heave_val = 0.25 * data.linear.z
		yaw_val = 0.25 * data.angular.z
		#print(surge_val)
		#print(sway_val)
		#print(heave_val)
		#print(yaw_val)
		
		if not self.is_simulation:
			#self.drone.motion.send_thruster_setpoint(surge=surge_val, sway=sway_val, heave=heave_val, yaw=yaw_val)
			self.drone.motion.surge = surge_val
			self.drone.motion.sway = yaw_val
			self.drone.motion.heave = heave_val
			self.drone.motion.yaw = sway_val

	def __init__(self, simulation = False):
		self.is_simulation = 0
		self.rate = rospy.Rate(10)
		if not self.is_simulation:
			self.drone = Pioneer()
			self.depth_information = rospy.Publisher("depth_data", Int32, queue_size=1)
        
		rospy.Subscriber("control_information", Twist, self.control_information_callback)
    
	def run(self):
		while not rospy.is_shutdown():
		    depth = self.drone.depth
			self.depth_information.publish(depth)
			self.rate.sleep()


if __name__ == "__main__":
	rospy.init_node("blueye_interface")
	#is_simulation = rospy.get_param('~simulation', False)

	try:
		interface = blueye_interface #(is_simulation)
		interface.run()
	except rospy.ROSInterruptException:
		pass

