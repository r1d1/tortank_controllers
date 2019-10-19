#!/usr/bin/env python

import argparse
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
    def __init__(self, plot):
        self.command_period = 0.1 # seconds
        self.min_dist = 0.2
        self.plot = plot
        self.vx = 0.0
        self.wz = 0.0
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
        proximities = [2.0*self.f_proximity((x_-self.min_dist)) for x_ in self.distances]
        influence = [self.sigmoid(2.0*(x_-1.0)) for x_ in proximities]
        projection_x = [x_*np.cos(-alpha_*np.pi/180.0) for x_,alpha_ in zip(influence, np.concatenate((np.arange(270,360), np.arange(0,90))))]
        projection_y = [x_*np.sin(alpha_*np.pi/180.0) for x_,alpha_ in zip(influence, np.concatenate((np.arange(270,360), np.arange(0,90))))]
        # divide by control period to get speed
        self.motors['l'] = np.dot(self.distances, self.weights['l']) #0.1+0.24*0.1*
        self.motors['r'] = np.dot(self.distances, self.weights['r']) #0.1+0.24*0.1*
        #plt.plot([self.motors['r'] if i < 90 else self.motors['l'] for i in np.arange(180)])
        self.vx=0.25-np.sum(projection_x)/180.
        self.wz=self.wz=-np.sum(projection_y)/180.

        if matplotlib_available and self.plot:
            plt.cla()
            plt.plot(self.distances, 'ko', label='distance')
            #plt.plot([self.sigmoid(x_)-0.5 for x_ in self.distances], 'o')
            plt.plot(proximities, 'o', label='prox')
            plt.plot(influence, 'ro', label='influence')
            plt.plot(projection_x, 'm', label='proj_x')
            plt.plot(projection_y, 'c', label='proj_y')
            #plt.plot(self.weights['r'], 'b')
            #plt.plot(self.weights['l'], 'r')

            plt.arrow(90.,1.,0.0,1.0)

            plt.legend()
            plt.draw()
            plt.pause(0.01)
    
    def timer_cb(self, event):
        # update speed
        msg_ = Twist()
        msg_.linear.x = self.vx
        msg_.angular.z = self.wz
        self.command_pub.publish(msg_)
        pass

    def stop_cb(self,msg):
        print("stop cb")
        msg_ = Twist()
        msg_.linear.x = 0.0
        msg_.angular.z = 0.0
        self.command_pub.publish(msg_)
    
if __name__ == "__main__":
    rospy.init_node('tortank')
    
    # get path from options:
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--plot", action="store_true", default=False)
    args=parser.parse_args()
    print(args)

    ctrl = BasicController(args.plot)
    if matplotlib_available and args.plot:
        plt.ion()
        plt.show()
    rospy.spin()

