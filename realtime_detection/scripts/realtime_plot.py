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
from std_msgs.msg import String, Float64
from realtime_detection.msg import Obstacle_Global_Position, Obstacles_Global_Position

def main():
    rospy.init_node('plot_obstacle', anonymous=True)
    plot_obstacle = Plot_Obstacle()
    plot_obstacle.loop()

class Plot_Obstacle:
    def __init__(self):
        self.bamboo = [507987.782655, 4285783.03875]
        self.iron = [507986.129448, 4285783.024]
        self.WGS84 = pyproj.Proj('+init=EPSG:4326')
        self.UTM54 = pyproj.Proj('+init=EPSG:32654')
        self.sub_obstacle_position = rospy.Subscriber("/realtime_detect/obstale_UTM", Obstacles_Global_Position, self.callback, queue_size = 1)

    def callback(self, data):
        self.obstacle_position = data

    def loop(self):
        fig, ax = plt.subplots(1,1)
        pole_x = [self.bamboo[0], self.iron[0]]
        pole_y = [self.bamboo[1], self.iron[1]]

        ax.scatter(pole_x, pole_y)

        #ax.set_xlim(507975,507995)
        #ax.set_ylim(4285765,4285785)

        while not rospy.is_shutdown():
            obstacle_position = self.obstacle_position
            num_of_objects = len(obstacle_position.obstacles_global_position)
            if obstacle_position.judge:
                for i in range(0, num_of_objects):
                    x = obstacle_position.obstacles_global_position[i].UTM_x
                    y = obstacle_position.obstacles_global_position[i].UTM_y
                    ax.scatter(x, y)
                    plt.pause(.01)

            else:
                pass



if __name__ == '__main__':
    main()
