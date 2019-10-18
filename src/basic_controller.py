#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt

class BasicController:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_cb)
        self.command_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        self.distances = None
        self.factors = {'r': [-0.05 * x for x in np.linspace(-90,90, num=180)], 'l': [0.05 * x for x in np.linspace(-90,90, num=180)]}
        print(self.factors['r'] / np.linalg.norm(self.factors['r']))
        self.weights = {'r': self.factors['r'] / np.linalg.norm(self.factors['r']), 'l': self.factors['l'] / np.linalg.norm(self.factors['l'])}
        self.motors = {'r': 0.0, 'l': 0.0}

    def laser_cb(self, msg):
        # store and process data
        self.distances = msg.ranges[270:360]+msg.ranges[0:90]
        plt.cla()
        plt.plot(self.distances, 'o')
        plt.plot(self.weights['r'], 'b')
        plt.plot(self.weights['l'], 'r')
        self.motors['l'] = 0.1+0.24*0.1*np.dot(self.distances, self.weights['l'])
        self.motors['r'] = 0.1+0.24*0.1*np.dot(self.distances, self.weights['r'])
        plt.plot([self.motors['r'] if i < 90 else self.motors['l'] for i in np.arange(180)])
        plt.draw()
        plt.pause(0.01)
    
    def timer_cb(self, event):
        # update speed
        #print('timer')
        pass
    
if __name__ == "__main__":
    rospy.init_node('tortank')
    
    ctrl = BasicController()
    plt.ion()
    plt.show()
    rospy.spin()

