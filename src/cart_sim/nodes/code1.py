#!/usr/bin/env python3

import rospy
from cart_sim.msg import cart_control
from sensor_msgs.msg import Joy

STEERING_AXIS = 0
THROTTLE_AXIS = 1

class Translator:
    def __init__(self):
        rospy.init_node('joy_translator')  # ROS düğümünü başlat
        self.sub = rospy.Subscriber("joy", Joy, self.callback)  # Joystick verileri için abone ol
        self.pub = rospy.Publisher('cart', cart_control, queue_size=1)  # Araba kontrol sinyalleri için yayımcı oluştur
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)  # Zamanlayıcı oluştur

    def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
            self.callback(self.last_published)

    def callback(self, message):
        rospy.logdebug("joy_translater received axes %s",message.axes)
        command = cart_control()
        command.header = message.header
        if message.axes[THROTTLE_AXIS] >= 0:
            command.throttle = message.axes[THROTTLE_AXIS]
            command.brake = 0.0
        else:
            command.brake = message.axes[THROTTLE_AXIS] * -1
            command.throttle = 0.0

        if message.buttons[3]:
            command.shift_gears = cart_control.FORWARD
        elif message.buttons[1]:
            command.shift_gears = cart_control.NEUTRAL
        elif message.buttons[0]:
            command.shift_gears = cart_control.REVERSE
        else:
            command.shift_gears = cart_control.NO_COMMAND

        command.steer = message.axes[STEERING_AXIS]
        self.last_published = message
        self.pub.publish(command)

if __name__ == '__main__':
    t = Translator()  # Translator sınıfından bir örnek oluştur
    rospy.spin()  # ROS düğümünün çalışmasını sürdür
