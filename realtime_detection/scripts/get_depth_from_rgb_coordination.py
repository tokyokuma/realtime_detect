#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def callback_depth(data, id):
    bridge = CvBridge()
    try:
        cv_image_depth = bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
        print e
    cv_image_depth = cv2.resize(cv_image_depth, (640, 360), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("depth", cv_image_depth)
    key = cv2.waitKey(delay=1)

    #rospy.loginfo(rospy.get_caller_id() + "\nqhd_depth_time:\nseq: [{}]\nsecs: [{}]\nnsecs: [{}]"
    #.format(data.header.seq, data.header.stamp.secs, data.header.stamp.nsecs))

def listner():
    rospy.init_node('get_depth', anonymous=True)
    image_sub_depth = rospy.Subscriber("/kinect2/qhd/image_depth_rect", Image, callback_depth, callback_args=0, queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    listner()
