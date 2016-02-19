#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import math
import rospy
import tf
import numpy as np


class gimbal(object):
    def __init__(self):
        rospy.init_node('gimbal')

        self.motor_sub = rospy.Subscriber("/motor_controller/state", JointState, self.motor_cb)
        self.motor_pub = rospy.Publisher("/motor_controller/command", Float64, queue_size=10)
        self.tf_pub = tf.TransformBroadcaster()

        # get parameters
        self.laser_radius = rospy.get_param('~pivot_radius', 0.0234)
        self.max_angle = rospy.get_param('~max_angle', 30.0)
        self.min_angle = rospy.get_param('~min_angle', -15.0)
        self.offset_angle = rospy.get_param('~offset_angle', 150.0)


        self.max_angle = np.deg2rad(self.max_angle)
        self.min_angle = np.deg2rad(self.min_angle)
        self.offset_angle = np.deg2rad(self.offset_angle)

        self.setpoint = self.min_angle + self.offset_angle

        rospy.spin()

    def motor_cb(self, data):
        theta = data.current_pos

        if not self.approx_equals(self.offset_angle + self.min_angle, self.setpoint):
            if self.approx_equals(self.offset_angle + self.max_angle, theta):
                self.setpoint = (self.offset_angle + self.min_angle)
        else:
            if self.approx_equals(self.offset_angle + self.min_angle, theta):
                self.setpoint = (self.offset_angle + self.max_angle)

        pitch = self.offset_angle - theta

        x = self.laser_radius * np.sin(pitch)
        y = 0.0
        z = self.laser_radius * np.cos(pitch)

        self.tf_pub.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(0, pitch, 0),
                                  rospy.Time.now(),
                                  "laser", "world")
        self.motor_pub.publish(self.setpoint)

    def approx_equals(self, setpoint, theta, tolerance = np.deg2rad(2.0)):
        error = np.abs(setpoint - theta)

        if error <= tolerance:
            return True
        else:
            return False



if __name__ == '__main__':
    node = gimbal()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
