#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag

bag_file_path = 'golf_odom.bag'
x_values = []
y_values = []

with rosbag.Bag(bag_file_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/base_pose_ground_truth']):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        x_values.append(x)
        y_values.append(y)

print(x_values)
