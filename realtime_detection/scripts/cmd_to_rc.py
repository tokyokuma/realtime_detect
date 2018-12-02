#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)
        # messageの型を作成
        self.RC_msg = OverrideRCIn()
        self.cmd_vel = Twist()

    def make_msg(self, data):
        # 処理を書く
        test = data.header
        print test
        #self.cmd_vel = cmd_vel
        #self.RC_msg.channels = [1500 - cmd_vel.angular.z*200, 1500 - cmd_vel.linear.x*150, 1500, 1500, 1500, 1500, 1500, 1500] #1:steering,2:thrust
	    #print(self.RC_msg.channels)

    def send_msg(self):
        # messageを送信
        self.make_msg(self.cmd_vel)
        self.publisher.publish(self.RC_msg)

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        #self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size = 1)
        self.Subscriber("/kinect2/qhd/image_color_rect", Image, self.callback, queue_size = 1)
        # messageの型を作成
        # publish
	    self.pub = pub

    def callback(self, data):
        #callback時の処理
        self.pub.make_msg(data)
        self.pub.send_msg()

def main():
    # nodeの立ち上げ
    rospy.init_node('cmd_to_RC')

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_publishers(pub)

    rospy.spin()

if __name__ == '__main__':
    main()
