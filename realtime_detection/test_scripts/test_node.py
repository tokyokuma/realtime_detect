#!/usr/bin/env python
import rospy
import sys
import cv2

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64, Header
from time import sleep

def main():
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, callback, queue_size = 1, buff_size= 2 ** 24)
    #rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, callback, queue_size = 1)
    #rospy.Subscriber('/mavros/imu/data', Imu, callback, queue_size = 1)
    rospy.spin()

def callback(data):
    '''
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print e
    '''

    r = rospy.Rate(5)
    r.sleep()

    #cv2.ims1w("RGB", cv_image)
    #key = cv2.waitKey(delay=1)

    test = data.header
    print test
    #pub_test.publish(test)

if __name__ == '__main__':
    pub_test = rospy.Publisher('/realtime_detect/test', Header, queue_size=1)
    main()
