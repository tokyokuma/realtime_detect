#!/usr/bin/env python
import rospy
import cv2
import sys
import pyproj
import math
import numpy as np
import matplotlib
import time
import matplotlib.pyplot as plt
matplotlib.use('Agg')
from std_msgs.msg import String, Float64
from realtime_detection.msg import Obstacle_Center, Obstacles_Center

def main():
    plot_obstacle = Plot_Obstacle()
    rospy.init_node('plot_obstacle', anonymous=True)


class Plot_Obstacle:
    def __init__(self):
        self.bridge = CvBridge()
        self.bamboo = [507987.782655, 4285783.03875]
        self.iron = [507986.129448, 4285783.024]
        self.WGS84 = pyproj.Proj('+init=EPSG:4326')
        self.UTM54 = pyproj.Proj('+init=EPSG:32654')
        self.sub_obstacle_position = rospy.Subscriber("/realtime_detect/obstale_UTM", Obstacles_Global_Position, self.callback, queue_size = 1)

    def callback(self, data):
        self.obstacle_position = self.sub_obstacle_position

    def loop(self):
        while not rospy.is_shutdown():
            obstacle_position = self.obstacle_position


    def Get_position(self, data, id):
        lon = data.longitude
        lat = data.latitude
        x, y = list(pyproj.transform(self.WGS84, self.UTM54, lon, lat))
        print 'x : ' + str(x) + '/ y : ' + str(y)
        plt.subplot(1,1,1),plt.scatter(x, y)
        plt.title('Position'),plt.xlim([507975,507995]),plt.ylim([4285765,4285785])
        plt.pause(0.000001)
        plt.subplot(1,1,1),plt.scatter(self.bamboo[0], self.bamboo[1])
        plt.pause(0.000001)
        plt.subplot(1,1,1),plt.scatter(self.iron[0], self.iron[1])
        plt.pause(0.000001)

    def Get_hdg(self, data, id):
        print data
        xy_start = np.array([0,-1])
        xy_finish = np.array([0,1])
        hdg = data.data
        compass_rad = math.radians(hdg)
        compass_rot_mat = np.array([[math.cos(compass_rad), math.sin(compass_rad)],
                                   [(-1)*math.sin(compass_rad), math.cos(compass_rad)]])
        xy_start_rot = np.dot(compass_rot_mat, xy_start.T)
        xy_finish_rot = np.dot(compass_rot_mat, xy_finish.T)
#        plt.clf()
#        plt.subplot(2,2,2),plt.quiver(xy_start_rot.item(0),xy_start_rot.item(1),
#                                      xy_finish_rot.item(0),xy_finish_rot.item(1),
#                                      angles='xy',scale_units='xy',scale=1)
#        plt.title('Hdg'),plt.xlim([-2,2]),plt.ylim([-2,2])
#        plt.pause(0.000001)

    def Get_depth(self, data, id):
        pass
if __name__ == '__main__':
    main()
