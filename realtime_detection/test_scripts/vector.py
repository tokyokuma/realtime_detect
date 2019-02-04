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
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge, CvBridgeError

def main():
    get_rbg = Get_sendor_data()
    rospy.init_node('get_sensor_data', anonymous=True)
    rospy.spin()
    cv2.destroyAllWindows()

def High_contrast(min, max, img):
    #re_contrast table
    min_table = min
    max_table = max
    diff_table = max_table - min_table

    LUT_HC = np.arange(256, dtype = 'uint8' )
    LUT_LC = np.arange(256, dtype = 'uint8' )

    # create high-contrast LUT
    for a in range(0, min_table):
        LUT_HC[a] = 0
    for a in range(min_table, max_table):
        LUT_HC[a] = 255 * (a - min_table) / diff_table
    for a in range(max_table, 255):
        LUT_HC[a] = 255

    return cv2.LUT(img, LUT_HC)

class Get_sendor_data:
    def __init__(self):
        self.bridge = CvBridge()
        self.bamboo = [507987.782655, 4285783.03875]
        self.iron = [507986.129448, 4285783.024]
        self.WGS84 = pyproj.Proj('+init=EPSG:4326')
        self.UTM54 = pyproj.Proj('+init=EPSG:32654')
        #self.image_sub_rgb = rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.Get_rgb, callback_args=0, queue_size = 1)
        #self.global_position = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.Get_position, callback_args=1, queue_size = 1)
        self.global_position = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.Get_hdg, callback_args=2, queue_size = 1)

    def Get_rgb(self, data, id):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        cv_image = cv2.resize(cv_image, (640, 360))
        cv_image = High_contrast(70, 200, cv_image)
        cv2.imshow('RGB', cv_image)
    #    position = rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, queue_size = 1)
    #    fix_position = self.Get_fix(self, position[0])

    #    print fix_position
        key = cv2.waitKey(delay=1)

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
        plt.clf()
        plt.subplot(1,1,1),plt.quiver(xy_start_rot.item(0),xy_start_rot.item(1),
                                      xy_finish_rot.item(0),xy_finish_rot.item(1),
                                      angles='xy',scale_units='xy',scale=1)
        plt.title('Hdg'),plt.xlim([-2,2]),plt.ylim([-2,2])
        plt.pause(0.000001)

    def Get_depth(self, data, id):
        pass
if __name__ == '__main__':
    main()
