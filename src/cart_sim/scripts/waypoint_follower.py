#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import rosbag
import matplotlib.pyplot as plt

class TurtlebotPID:
    def __init__(self):
        rospy.init_node('turtlebot_pid_controller')
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
        self.rate = rospy.Rate(50)
        self.cmd_vel = Twist()

        self.target_position = Point()
        self.current_position = Point()
        self.current_yaw = 0

        self.kp = 0.05
        self.ki = 0.01
        self.kd = 0.1

        self.prev_error = 0
        self.integral = 0

        self.fig, self.ax = plt.subplots()
        self.points = []
        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

        self.ax.plot(self.current_position.x, self.current_position.y, 'bo')

    def calculate_distance(self):
        distance=math.sqrt((self.target_position.x-self.current_position.x)**2+(self.target_position.y-self.current_position.y)**2)
        return distance

    def calculate_heading(self):
        target_yaw = math.atan2(self.target_position.y - self.current_position.y,
                                self.target_position.x - self.current_position.x)
        heading = target_yaw - self.current_yaw
        return heading

    def pid_control(self):
        distance = self.calculate_distance()
        heading = self.calculate_heading()

        proportional = distance
        self.integral = distance
        derivative = distance - self.prev_error

        control_output=0

        control_output =self.kp * proportional + self.ki * self.integral + self.kd * derivative

        self.cmd_vel.linear.x = control_output*10
        self.cmd_vel.angular.z = heading

        self.vel_pub.publish(self.cmd_vel)

        self.prev_error = distance
        self.rate.sleep()

    def show_plot(self):
        plt.xlabel('X Koordinatı')
        plt.ylabel('Y Koordinatı')
        plt.title('TurtleBot3 Yolu')
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def draw_points(self):
        for point in self.points:
            self.ax.plot(point[0], point[1], 'bo')
        self.ax.plot(self.current_position.x, self.current_position.y, 'ro')
        self.ax.figure.canvas.draw()


    def on_press(self, event):
        if event.button == 1:
            self.points.append((event.xdata, event.ydata))
            self.draw_points()
        elif event.button == 3:
            if not self.points:
                return

            min_distance = float('inf')
            closest_index = None
            for i, point in enumerate(self.points):
                distance = ((point[0] - event.xdata) ** 2 + (point[1] - event.ydata) ** 2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    closest_index = i

            if min_distance < 1:
                del self.points[closest_index]
                self.draw_points()

        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)

    def on_release(self, event):
        if event.button == 3:
            if not self.points:
                return

            min_distance = float('inf')
            closest_index = None
            for i, point in enumerate(self.points):
                distance = ((point[0] - event.xdata) ** 2 + (point[1] - event.ydata) ** 2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    closest_index = i

            if min_distance < 1:
                del self.points[closest_index]
                self.draw_points()


if __name__ == '__main__':
    bag_file_path = '/home/talos-baris/turtlebot3_odom.bag'
    x_values = []
    y_values = []

    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/odom']):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            x_values.append(x)
            y_values.append(y)

    i=0
    try:
        pid_controller = TurtlebotPID()
        while i < len(x_values):
            dist=pid_controller.calculate_distance()
            print(dist,i)
            if dist<0.1:
                pid_controller.target_position.x = x_values[i]
                pid_controller.target_position.y = y_values[i]
                i = i+10
            else:
                pid_controller.target_position.x = x_values[i]
                pid_controller.target_position.y = y_values[i]
            pid_controller.pid_control()
            i += 2

        pid_controller.draw_points()
        pid_controller.show_plot()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS düğümü sonlandırıldı.")