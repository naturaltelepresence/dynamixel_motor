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
        self.frame_id = rospy.get_param('~frame_id', 'gimbal')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'laser')
        self.tol = np.deg2rad(5.0)


        self.max_angle = np.deg2rad(self.max_angle)
        self.min_angle = np.deg2rad(self.min_angle)
        self.offset_angle = np.deg2rad(self.offset_angle)

        assert self.max_angle > self.min_angle

        self.setpoint = self.offset_angle + self.max_angle + self.tol

        rospy.spin()

    def motor_cb(self, data):
        theta = data.current_pos

        if theta > (self.offset_angle + self.max_angle):
            self.setpoint = self.offset_angle + self.min_angle - self.tol
        elif theta < (self.offset_angle + self.min_angle):
            self.setpoint = self.offset_angle + self.max_angle + self.tol

        pitch = self.offset_angle - theta

        x = self.laser_radius * np.sin(pitch)
        y = 0.0
        z = self.laser_radius * np.cos(pitch)

        self.tf_pub.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(0, pitch, 0),
                                  rospy.Time.now(),
                                  self.child_frame_id, self.frame_id)
        self.motor_pub.publish(self.setpoint)

if __name__ == '__main__':
    node = gimbal()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
