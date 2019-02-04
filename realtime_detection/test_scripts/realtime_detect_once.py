#!/usr/bin/env python
import rospy
import cv2
import math
import pyproj
import numpy as np
import time
import message_filters
from PIL import Image
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from realtime_detection.msg import Obstacle_Center, Obstacles_Center

def callback(NavSatFix, Float64, Imu):
    gps_time = float(NavSatFix.header.stamp.secs) + float(NavSatFix.header.stamp.nsecs) * 10 ** -9
    imu_time = float(Imu.header.stamp.secs) + float(Imu.header.stamp.nsecs) * 10 ** -9
    diff_time = gps_time - imu_time
    print diff_time

def listener():
    rospy.init_node('listener', anonymous=True)

    center_sub = message_filters.Subscriber("/realtime_detect/obstale_detection", Obstacles_Center)
    gps_sub = message_filters.Subscriber('/mavros/global_position/global', NavSatFix)
    #hdg_sub = message_filters.Subscriber("/mavros/global_position/compass_hdg", Float64)
    imu_sub = message_filters.Subscriber('/mavros/imu/data', Imu)
    ts = message_filters.ApproximateTimeSynchronizer([center_sub, gps_sub, imu_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
i
