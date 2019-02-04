#!/usr/bin/env python
from __future__ import division
from __future__ import print_function
import rospy
import os
import cv2
import torch
import numpy as np
import time
import SegmentationModel as net
import fast_ESPNetv2 as fast
from torch import nn
from PIL import Image
from argparse import ArgumentParser
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64
from realtime_detection.msg import Obstacle_Center, Obstacles_Center

pallete = [[153,153,153],
           [170,234,150],
           [220,220,  0],
           [107,142, 35],
           [152,251,152],
           [ 70,130,180],
           [220, 20, 60],
           [  0, 60,100],
           [150,250,250],
           [  0,  0,  0],
           [  0,  0,  0]]

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
            print ('Cv_Brdige_Error')

    def loop(self):
        modelA = net.EESPNet_Seg(args.classes, s=args.s)
        if not os.path.isfile(args.pretrained):
            print('Pre-trained model file does not exist. Please check ./pretrained_models folder')
            exit(-1)

        modelA = nn.DataParallel(modelA)
        modelA.load_state_dict(torch.load(args.pretrained))
        modelA = modelA.cuda()
        fmt = cv2.VideoWriter_fourcc(*'XVID')
        fps = 6.0
        size = (768, 432*2)
        writer = cv2.VideoWriter('/media/nvidia/outtest.avi', fmt, fps, size)
        # set to evaluation mode
        modelA.eval()

        while not rospy.is_shutdown():
            try:
                self.cv_image
            except AttributeError:
                continue

            start = time.time()

            cv_image = self.cv_image
            data_header = self.data_header
            #cv_image = Sharp(cv_image)
            #cv_image = High_contrast(50, 200, cv_image)
            img = cv2.resize(cv_image, (args.width, args.height))
            if args.colored:
                if args.overlay:
                    gray, segmentation, overlay = evaluateModel(args, modelA, img)
                else:
                    gray, segmentation = evaluateModel(args, modelA, img)
            else:
                gray = evaluateModel(args, modelA, img)

            gray_orig = np.copy(gray)
            pole_threshold = Pole_Threshold(gray)
            pole_remove_noise = Open_Close(pole_threshold)
            id_and_center = Calc_Center(pole_remove_noise)

            num_of_objects = len(id_and_center)

            detect_info = Obstacles_Center()
            detect_info.header = data_header
            obj = Obstacle_Center()

            for id in range(0, num_of_objects):
                obj.Class = 'obstacle'
                obj.center_y = id_and_center[id][0]
                obj.center_x = id_and_center[id][1]
                detect_info.obstacles_center.append(obj)

            im_h = cv2.vconcat([img, segmentation])
            #print (im_h.shape)
            writer.write(im_h)
            #cv2.imshow("RGB_seg", im_h)

            #cv2.imshow("RGB", img)
            #cv2.imshow("gray", gray_orig)
            #cv2.imshow("pole_detect", pole_remove_noise)
            #cv2.imshow("segmentation", segmentation)
            #cv2.imshow("overlay", overlay)

            elapsed_time = time.time() - start
            print (elapsed_time)
            fps = float(1.0/elapsed_time)

            self.pub_obstacle_center.publish(detect_info)
            self.pub_FPS.publish(Float64(fps))

            key = cv2.waitKey(delay=1)

        writer.release()

def evaluateModel(args, model, img):
    # gloabl mean and std values
    mean = np.array([131.84157, 145.38597, 135.16437])
    std = np.array([76.013596, 67.85283,  70.89791 ])

    model.eval()
    img_orig = np.copy(img)

    #0.015sec
    img = img.astype(np.float32)
    fast.mean_std(img, mean, std)

    #0.01
    img /= 255
    img = img.transpose((2, 0, 1))
    img_tensor = torch.from_numpy(img)
    img_tensor = torch.unsqueeze(img_tensor, 0)  # add a batch dimension
    img_tensor = img_tensor.cuda()

    #0.088
    img_out = model(img_tensor)
    classMap_numpy = img_out[0].max(0)[1].byte().cpu().data.numpy()
    classMap_numpy = classMap_numpy.astype(np.uint8)

    if args.colored:
        classMap_numpy_color = np.zeros((img.shape[1], img.shape[2], img.shape[0]), dtype=np.uint8)
        for idx in range(len(pallete)):
            [r, g, b] = pallete[idx]
            classMap_numpy_color[classMap_numpy == idx] = [b, g, r]
        if args.overlay:
            overlayed = cv2.addWeighted(img_orig, 0.5, classMap_numpy_color, 0.5, 0)

            return classMap_numpy, classMap_numpy_color, overlayed
        else:
            return classMap_numpy, classMap_numpy_color
    else:
        return classMap_numpy

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
    element8 = np.ones((1, 1), np.uint8)
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

def Sharp(img):
    #sharpened
    k = 1.0
    op = np.array([[-k, k, -k],
                   [-k, 1 + 6 * k, -k],
                   [-k, -k, -k]])

    img_tmp = cv2.filter2D(img, -1 , op)
    sharp_img = cv2.convertScaleAbs(img_tmp)

    return sharp_img

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

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--model', default="ESPNetv2", help='Model name')
    parser.add_argument('--width', type=int, default=768, help='Width of RGB image')
    parser.add_argument('--height', type=int, default=432, help='Height of RGB image')
    parser.add_argument('--pretrained', default='/home/nvidia/tools/ESPNetv2/models/izunuma_dataset9_0.5/model_best.pth', help='Pretrained weights directory.')
    parser.add_argument('--s', default=0.5, type=float, help='scale')
    parser.add_argument('--colored', default=False, type=bool, help='If you want to visualize the '
                                                                   'segmentation masks in color')
    parser.add_argument('--overlay', default=False, type=bool, help='If you want to visualize the '
                                                                   'segmentation masks overlayed on top of RGB image')
    parser.add_argument('--classes', default=11, type=int, help='Number of classes in the dataset. 20 for Cityscapes')
    args = parser.parse_args()
    main()
