#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

matplotlib_available = False
try:
    import matplotlib.pyplot as plt
    matplotlib_available = True
except ImportError:
    print("No matplotlib, no plotting")


class BasicController:
    def __init__(self):
        self.command_period = 0.1 # seconds
        self.min_dist = 0.3
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_cb)
        self.stop_sub = rospy.Subscriber('stop_robot', Empty, self.stop_cb)
        self.command_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.command_period), self.timer_cb)
        
        self.distances = None
        self.factors = {'r': [-0.05 * x for x in np.linspace(-90,90, num=180)], 'l': [0.05 * x for x in np.linspace(-90,90, num=180)]}
        print(self.factors['r'] / np.linalg.norm(self.factors['r']))
        self.weights = {'r': self.factors['r'] / np.linalg.norm(self.factors['r']), 'l': self.factors['l'] / np.linalg.norm(self.factors['l'])}
        self.motors = {'r': 0.0, 'l': 0.0}

    def sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-x))

    def f_proximity(self, x):
        return np.exp(-x)

    def laser_cb(self, msg):
        # store and process data
        self.distances = msg.ranges[270:360]+msg.ranges[0:90]

        # divide by control period to get speed
        self.motors['l'] = np.dot(self.distances, self.weights['l']) #0.1+0.24*0.1*
        self.motors['r'] = np.dot(self.distances, self.weights['r']) #0.1+0.24*0.1*
        #plt.plot([self.motors['r'] if i < 90 else self.motors['l'] for i in np.arange(180)])
        if matplotlib_available :
            plt.cla()
            plt.plot(self.distances, 'ko')
            #plt.plot([self.sigmoid(x_)-0.5 for x_ in self.distances], 'o')
            plt.plot([2.0*self.f_proximity(x_-self.min_dist) for x_ in self.distances], 'o')
            #plt.plot(self.weights['r'], 'b')
            #plt.plot(self.weights['l'], 'r')
            plt.draw()
            plt.pause(0.01)
    
    def timer_cb(self, event):
        # update speed
        #print('timer')
        #print(self.motors)
        msg = Twist()
        self.command_pub.publish(msg)

    def stop_cb(self,msg):
        print("stop cb")
        msg_ = Twist()
        msg_.linear.x = 0.0
        msg_.angular.z = 0.0
        self.command_pub.publish(msg_)
    
if __name__ == "__main__":
    rospy.init_node('tortank')
    
    ctrl = BasicController()
    if matplotlib_available :
        plt.ion()
        plt.show()
    rospy.spin()

