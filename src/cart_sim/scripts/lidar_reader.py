#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2



class LidarReader:
    def __init__(self):
        rospy.Subscriber("/cart/center_laser/scan", PointCloud2, self.callback)

    def callback(self, data):
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = []
        for p in pc:
            pc_list.append([p[0],p[1],p[2]])
            print(pc_list)

if __name__ == "__main__":
    try:
        rospy.init_node("lidar_reader")
        l = LidarReader()
        rospy.spin()
    except:
        print("shutting down")