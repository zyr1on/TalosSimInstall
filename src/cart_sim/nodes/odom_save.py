#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import rosbag
from cart_sim.msg import cart_control

rospy.init_node('base_pose_ground_truth')

bag_file_path = 'my_bag.bag'

bag = rosbag.Bag(bag_file_path, 'w')


def cart_control_callback(msg):
    twist_msg = Twist()
    twist_msg.linear.x = msg.speed
    twist_msg.angular.z = msg.steer_angle
    bag.write('/cmd_vel', twist_msg)

rospy.Subscriber('/cart', cart_control, cart_control_callback)

rospy.spin()

bag.close()
