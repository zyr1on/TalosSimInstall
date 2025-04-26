#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from cart_sim.msg import cart_control
import numpy as np
import rosbag
from sklearn.preprocessing import MinMaxScaler

class TurtlebotPID:
    def __init__(self):
        rospy.init_node('turtlebot_pid_controller')
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cart', cart_control, queue_size=50)
        self.rate = rospy.Rate(50)
        self.target_position = Point()
        self.current_position = Point()
        self.current_yaw = 0
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1
        self.prev_error = 0
        self.integral = 0

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

    def calculate_distance(self):
        distance = math.sqrt((self.target_position.x - self.current_position.x) ** 2 +(self.target_position.y - self.current_position.y) ** 2)
        return distance

    def calculate_heading(self):
        delta_x = self.target_position.x - self.current_position.x
        delta_y = self.target_position.y - self.current_position.y
        target_yaw = math.atan2(delta_y, delta_x)

        heading = target_yaw - self.current_yaw
        print(heading)
        return heading
    
    def pid_control(self):
        distance = self.calculate_distance()
        heading = self.calculate_heading()

        error = distance
        self.integral += error
        derivative = error - self.prev_error

        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative

        throttle = np.clip(control_output, -1, 1) /10
        steer = np.clip(heading, -1, 1)

        command = cart_control()
        command.header.stamp = rospy.Time.now()
        command.shift_gears = cart_control.FORWARD

        if distance > 1:
            command.steer = steer
            command.throttle = throttle
            command.brake = 0
        else:
            command.steer = steer
            command.throttle = throttle
            command.brake = 1

        self.vel_pub.publish(command)

        self.prev_error = error
        self.rate.sleep()

if __name__ == '__main__':
    bag_file_path = '/home/talos-baris/golf_odom.bag'
    x_values = []
    y_values = []

    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/base_pose_ground_truth']):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            x_values.append(x)
            y_values.append(y)

    i = 10
    try:
        rospy.init_node('turtlebot_pid_controller')
        pid_controller = TurtlebotPID()
        
        while not rospy.is_shutdown() and i < len(x_values):
            dist = pid_controller.calculate_distance()
            if dist < 1:
                pid_controller.target_position.x = x_values[i]
                pid_controller.target_position.y = y_values[i]
                i += 1
            else:
                pid_controller.target_position.x = x_values[i]
                pid_controller.target_position.y = y_values[i]

            pid_controller.pid_control()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS düğümü sonlandırıldı.")
