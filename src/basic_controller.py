#/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class BasicController():
	def__init__(self):
		self.laser_sub = rospy.Subscriber('scan', self.laser_cb, queue_size=1)
		self.command_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def laser__cb(self, msg):
		# store and process data
		pass

	def timer_cb(self, event):
		# update speed
		pass


if __name__ == "__main__":
	rospy.init_node('tortank')

	ctrl = BasicController()
	rospy.spin()
