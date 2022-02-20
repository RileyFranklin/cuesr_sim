#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry

class OdomFix(object):

    def __init__(self):
        rospy.init_node('odom_fix')
        self.pub = rospy.Publisher('/drone/odom_fix', Odometry, queue_size=10)
        self.sub = rospy.Subscriber("/drone/mavros/odometry/in", Odometry, self.callback)

    def callback(self, msg):
        cov = np.array(msg.twist.covariance)
        cov[3*6 + 3] = 1e-3 
        cov[4*6 + 4] = 1e-3
        cov[5*6 + 5] = 1e-3
        msg.twist.covariance = cov
        self.pub.publish(msg)

OdomFix()
rospy.spin()

