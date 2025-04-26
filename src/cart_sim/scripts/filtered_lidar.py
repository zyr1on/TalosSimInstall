import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

#struct error alıyorsan çalıştır
#/sensor_stick/point_cloud node üzerinden gelen lidar verisindeki 
#pcl_helper için datanın width ve height değişkenlerinin yerini değiştir 
class LidarReader:
    def __init__(self):
        rospy.Subscriber("/sensor_stick/point_cloud", PointCloud2, self.callback)
        self.pub = rospy.Publisher("my_filtered_cloud", PointCloud2, queue_size=10) 
        
    def callback(self, data):
        #data.width <--> data.height
        tmp = data.width
        data.width = data.height
        data.height = tmp
        #publish from my_filtered_cloud
        self.pub.publish(data)

if __name__ == "__main__":
    try:
        rospy.init_node("lidar_reader")
        l = LidarReader()
        rospy.spin()
    except:
        print("shutting down")