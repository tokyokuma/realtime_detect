#!/usr/bin/env python
import rospy
import cv2
import caffe
import numpy as np
import calc_area as ca
import time
from PIL import Image
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64
from realtime_detection.msg import Obstacle_Center, Obstacles_Center


def main():
    rospy.init_node('detect_obstacle', anonymous=True)
    realtime_detect = Realtime_Detect()
    realtime_detect.loop()

class Realtime_Detect():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.callback, queue_size = 1, buff_size=2**24)
        self.pub_obstacle_center = rospy.Publisher('/realtime_detect/obstale_detection', Obstacles_Center, queue_size = 1)
        self.pub_FPS = rospy.Publisher('/realtime_detect/FPS', Float64, queue_size = 1)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.data_header = data.header

        except CvBridgeError as e:
            print "Cv_Brdige_Error"

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.cv_image
            except AttributeError:
                continue

            start = time.time()

            cv_image = self.cv_image
            data_header = self.data_header

            caffe.set_mode_gpu()
            cv_image = cv2.resize(cv_image, (640, 360))
            in_ = np.array(cv_image, dtype=np.float32)
            in_ = in_.transpose((2,0,1))
            net.blobs['data'].reshape(1, *in_.shape)
            net.blobs['data'].data[...] = in_
            net.forward()
            out = net.blobs['score'].data[0].argmax(axis=0)
            #out = net.blobs['deconv6_0_0'].data[0].argmax(axis=0)
            seg_img_gray = np.array(out, dtype=np.uint8)
            seg_img = np.array(palette, dtype=np.uint8)[out]

            pole_threshold = Pole_Threshold(seg_img_gray)
            pole_remove_noise = Open_Close(pole_threshold)
            id_and_center = Calc_Center(pole_remove_noise)

            num_of_objects = len(id_and_center)

            detect_info = Obstacles_Center()
            detect_info.header = data_header
            obj = Obstacle_Center()

            for id in range(0, num_of_objects):
                obj.Class = 'pole'
                obj.center_y = id_and_center[id][0]
                obj.center_x = id_and_center[id][1]
                detect_info.obstacles_center.append(obj)

            #cv2.imshow("RGB", cv_image)
            #cv2.imshow("pole_detect", pole_remove_noise)
            #cv2.imshow("segmentation", seg_img)

            elapsed_time = time.time() - start
            fps = float(1.0/elapsed_time)

            self.pub_obstacle_center.publish(detect_info)
            self.pub_FPS.publish(Float64(fps))

            key = cv2.waitKey(delay=1)

def Pole_Threshold(img):
    #0.0011sec
    recolor_img = img.copy()
    pole = [0]
    black = [0]
    white = [255]
    recolor_img[np.where(recolor_img == pole)] = white
    recolor_img[np.where(recolor_img != white)] = black

    return recolor_img

def Open_Close(img):
    #0.0040sec
    img_tmp = img
    element8 = np.ones((10, 10), np.uint8)
    iteration = 1
    while iteration <= 1:
        img_tmp = cv2.morphologyEx(img_tmp, cv2.MORPH_OPEN, element8)
        img_tmp = cv2.morphologyEx(img_tmp, cv2.MORPH_CLOSE, element8)
        iteration = iteration + 1
    return img_tmp

def Calc_Center(pole_remove_noise):
    #0.0083sec
    label = cv2.connectedComponentsWithStats(pole_remove_noise)
    n = label[0] - 1
    #remove backgrouond label
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    obstacle_coordination = np.zeros([n,2], dtype=np.float32)

    for i in range(n):
        #obstacle_coordination[ID][y,x]
        obstacle_coordination[i][0] = int(center[i][1])
        obstacle_coordination[i][1] = int(center[i][0])

    return obstacle_coordination

if __name__ == '__main__':
    prototxt = '/home/nvidia/tools/caffe-enet/models/ENet/prototxt/enet_deploy_encoder_crop.prototxt'
    caffemodel = '/home/nvidia/tools/caffe-enet/models/ENet/caffemodel/enet_fine_izunuma_encoder_crop_dataset7_439914.caffemodel'
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    #net = caffe.Net('/home/nvidia/tools/caffe-enet/models/ENet/prototxt/enet_deploy_from_encoder_decoder.prototxt', '/home/nvidia/tools/caffe-enet/models/ENet/caffemodel/enet_fine_izunuma_decoder_3.caffemodel', caffe.TEST)
    palette = [(153,153,153),(153,234,170),(0,220,220),(35,142,107),(152,251,152),(180,130,70),(60,20,220),(100,60,0),(250,250,250),(128,128,128)]
    #palette = [(90,120,150),(153,153,153),(153,234,170),(35,142,107),(152,251,152),(180,130,70),(60,20,220),(100,60,0),(128,128,128)]

    main()
