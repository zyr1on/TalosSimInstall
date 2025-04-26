import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import rospy
from sensor_msgs.msg import LaserScan
from matplotlib.animation import FuncAnimation

EXTEND_AREA = 30.0  # [m] grid map extension length

show_animation = True

def generate_gaussian_grid_map(ox, oy, xyreso, std):
    minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(ox, oy, xyreso)

    gmap = [[0.0 for _ in range(yw)] for _ in range(xw)]
    for ix in range(xw):
        for iy in range(yw):
            x = ix * xyreso + minx
            y = iy * xyreso + miny

            # Search minimum distance
            mindis = float("inf")
            for (iox, ioy) in zip(ox, oy):
                d = math.hypot(iox - x, ioy - y)
                if mindis >= d:
                    mindis = d

            pdf = (1.0 - norm.cdf(mindis, 0.0, std))
            gmap[ix][iy] = pdf
    return gmap, minx, maxx, miny, maxy

def calc_grid_map_config(ox, oy, xyreso):
    minx = round(min(ox) - EXTEND_AREA / 2.0)
    miny = round(min(oy) - EXTEND_AREA / 2.0)
    maxx = round(max(ox) + EXTEND_AREA / 2.0)
    maxy = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))
    return minx, miny, maxx, maxy, xw, yw

def draw_heatmap(data, minx, maxx, miny, maxy, xyreso):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")

def process_laserscan(scan):
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment
    ranges = scan.ranges

    x_coords = []
    y_coords = []
    
    for i, r in enumerate(ranges):
        if math.isinf(r):
            continue 
        angle = angle_min + i * angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        x_coords.append(x)
        y_coords.append(y)
    return np.array(x_coords), np.array(y_coords)

class LaserScanProcessor:
    def __init__(self):
        self.ox = np.array([])
        self.oy = np.array([])
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.xyreso = 0.5
        self.std = 1.0

    def callback(self, scan):
        self.ox, self.oy = process_laserscan(scan)

    def update_plot(self, frame):
        gmap, minx, maxx, miny, maxy = generate_gaussian_grid_map(self.ox, self.oy, self.xyreso, self.std)
        plt.cla()
        draw_heatmap(gmap, minx, maxx, miny, maxy, self.xyreso)
        plt.plot(self.ox, self.oy, "xr")
        plt.plot(0.0, 0.0, "ob")
        plt.draw()

if __name__ == '__main__':
    rospy.init_node('laser_listener')
    processor = LaserScanProcessor()

    fig = plt.figure()
    ani = FuncAnimation(fig, processor.update_plot, interval=100)  
    plt.show()
    rospy.spin()
