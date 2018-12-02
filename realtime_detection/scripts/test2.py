#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64, Header
from time import sleep


class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)

    def make_msg(self, data):
        # 処理を書く
        test = data.header
        print test
        #self.cmd_vel = cmd_vel
        #self.RC_msg.channels = [1500 - cmd_vel.angular.z*200, 1500 - cmd_vel.linear.x*150, 1500, 1500, 1500, 1500, 1500, 1500] #1:steering,2:thrust
	    #print(self.RC_msg.channels)


class Subscribe_publishers():
    def __init__(self, pub):
        rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.callback, queue_size = 1, buff_size= 2 ** 24)
        self.data = 0
        self.pub = pub

    def callback(self, data):
        #callback時の処理
        #self.pub.make_msg(data)
        self.data = data.header

    def loop(self):
        #self.pub.publish(self.data)
        #self.pub.make_msg(self.data)
        while not rospy.is_shutdown():
            r.sleep()
            print self.data

def main():
    # nodeの立ち上げ

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    sub.loop()

if __name__ == '__main__':
    rospy.init_node('cmd_to_RC')
    r = rospy.Rate(3)
    main()
