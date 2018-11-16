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

def main():
    Get_rbg = get_rgb()
    rospy.init_node('get_rgb', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

def calculation_area(img):
    class_area = [0,0,0,0,0,0,0,0,0]
    all_class_num = len(class_list)
    height, width = img.shape
    all_pixels = height * width

    for y in range(0, height):
        for x in range(0, width):
            pixel = img.item(y,x)
            for i in range(0, all_class_num):
                if pixel == i:
                    class_area[i] = class_area[i] + 1
                else:
                    pass

    for j in range(0, all_class_num):
        class_area[j] = float(class_area[j]) / float(all_pixels) * 100
        #print class_list[j] + ' : ' + str(class_area[j]) + '%'

class get_rgb:
    def __init__(self):
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size = 1)
        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.callback, queue_size = 1)

    def callback(self, data):
        #start = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        caffe.set_mode_gpu()
        cv_image = cv2.resize(cv_image, (640, 360))
        in_ = np.array(cv_image, dtype=np.float32)
        in_ = in_.transpose((2,0,1))
        net.blobs['data'].reshape(1, *in_.shape)
        net.blobs['data'].data[...] = in_
        net.forward()
        #out = net.blobs['score'].data[0].argmax(axis=0)
        out = net.blobs['deconv6_0_0'].data[0].argmax(axis=0)

        seg_img_gray = np.array(out, dtype=np.uint8)
        seg_img = np.array(palette, dtype=np.uint8)[out]

        start = time.time()
        #calculation_area(out)
        #ca.calc_area(out)
        elapsed_time = time.time() - start

        cv2.imshow("RGB", cv_image)
        cv2.imshow("segmentation_gray", seg_img_gray)
        cv2.imshow("segmentation", seg_img)

        print elapsed_time
        fps = float(1.0/elapsed_time)
        print str(fps) + 'FPS'

        key = cv2.waitKey(delay=1)


if __name__ == '__main__':
    '''
    prototxt = '/home/nvidia/tools/caffe-enet/models/ENet/prototxt/enet_deploy_encoder_crop_final.prototxt'
    caffemodel = '/home/nvidia/tools/caffe-enet/models/ENet/caffemodel/enet_fine__fine_izunuma_encoder_crop_dataset6_73677_bn_conv_merged_weights.caffemodel'
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)
    '''
    class_list = ['bambu_pole', 'iron_pole', 'wood', 'lotus', 'chestnuta', 'sky', 'person', 'boat', 'water_sur']
    net = caffe.Net('/home/nvidia/tools/caffe-enet/models/ENet/prototxt/enet_deploy_from_encoder_decoder.prototxt', '/home/nvidia/tools/caffe-enet/models/ENet/caffemodel/enet_fine_izunuma_decoder_3.caffemodel', caffe.TEST)
    #palette = [(90,120,150),(153,153,153),(153,234,170),(0,220,220),(35,142,107),(152,251,152),(180,130,70),(60,20,220),(100,60,0),(250,250,250),(128,128,128)]
    palette = [(90,120,150),(153,153,153),(153,234,170),(35,142,107),(152,251,152),(180,130,70),(60,20,220),(100,60,0),(128,128,128)]

    main()
