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
from realtime_detection.msg import Obstacle_Global_Position, Obstacles_Global_Position

def main():
    rospy.init_node('calc_obstacle_coordination', anonymous=True)
    obstacle_coordination = Obstacle_Coordination()
    obstacle_coordination.loop()

class Obstacle_Coordination():
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_deg = 15
        self.camera_rad = math.radians(self.camera_deg)
        self.camera_x_from_GPS = -0.2975
        self.camera_y_from_GPS = 1.55
        self.camera_xy_from_GPS = np.array([self.camera_x_from_GPS, self.camera_y_from_GPS])
        self.fx=521.0852017415114
        self.fy=521.7842912470452
        self.cx=481.64180763282405
        self.cy=277.6362107374241
        self.r = rospy.Rate(5)
        self.depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)
        self.center_sub = message_filters.Subscriber('/realtime_detect/obstale_detection', Obstacles_Center)
        self.gps_sub = message_filters.Subscriber('/mavros/global_position/global', NavSatFix)
        self.imu_sub = message_filters.Subscriber('/mavros/imu/data', Imu)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.center_sub, self.gps_sub, self.imu_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        self.pub_obstacles_global_position = rospy.Publisher('/realtime_detect/obstale_UTM', Obstacles_Global_Position, queue_size = 1)

    def callback(self, Image, Obstacles_Center, NavSatFix, Imu):
        try:
            self.cv_image_depth = self.bridge.imgmsg_to_cv2(Image, "16UC1")
            self.center_header = Obstacles_Center.header
            self.center = Obstacles_Center.obstacles_center
            self.Pixhawk_Lon = NavSatFix.longitude
            self.Pixhawk_Lat = NavSatFix.latitude
            self.quaternion = (Imu.orientation.x, Imu.orientation.y, Imu.orientation.z, Imu.orientation.w)

        except CvBridgeError as e:
            print "Cv_Brdige_Error"

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.cv_image_depth
            except AttributeError:
                continue

            cv_image_depth = self.cv_image_depth
            center_header = self.center_header
            center = self.center

            Pixhawk_Lon = self.Pixhawk_Lon
            Pixhawk_Lat = self.Pixhawk_Lat
            quaternion = self.quaternion

            cv_image_depth_resize = cv2.resize(cv_image_depth, (768,432), interpolation=cv2.INTER_NEAREST)
            global_position_info = Obstacles_Global_Position()
            global_position_info.header = center_header

            euler = euler_from_quaternion(quaternion)

            roll_rad = math.radians(math.degrees(euler[0]))
            pitch_rad = math.radians(math.degrees(euler[1]))
            yaw_rad = math.radians(math.degrees(euler[2]))

            self.Pub_obstacle_global_position(roll_rad, pitch_rad, yaw_rad, Pixhawk_Lon, Pixhawk_Lat, center, cv_image_depth_resize, global_position_info)
            self.r.sleep()

    def Pub_obstacle_global_position(self, roll_rad, pitch_rad, yaw_rad, Pixhawk_Lon, Pixhawk_Lat, center, cv_image_depth_resize, global_position_info):
        camera_UTM_xy = self.Calc_camera_UTM(yaw_rad, Pixhawk_Lon, Pixhawk_Lat)
        obstacle_UTM_xyz = np.empty(3)
        num_of_object = len(center)

        for obstacle_id in range(0, num_of_object):
            depth = self.Get_depth(obstacle_id, center, cv_image_depth_resize)
            if depth != None:
                start = time.time()
                obj = Obstacle_Global_Position()
                obstacle_raw_XYZ3D = self.Convert_camera_to_world(obstacle_id, center, depth)
                XYZ3D_after_rotation = self.Imu_Camera_rotation(obstacle_raw_XYZ3D, roll_rad, pitch_rad)
                obj.UTM_x = XYZ3D_after_rotation[0] + camera_UTM_xy[0]
                #obj.UTM_y = XYZ3D_after_rotation[2] + camera_UTM_xy[1]
                obj.UTM_y = XYZ3D_after_rotation[2] + camera_UTM_xy[1] + 1.0

                obj.Height = 1.0 + XYZ3D_after_rotation[1]
                obj.Class = 'pole'

                global_position_info.obstacles_global_position.append(obj)
                global_position_info.judge = 1
                Pixhawk_UTM_x, Pixhawk_UTM_y = pyproj.transform(WGS84, UTM54, Pixhawk_Lon, Pixhawk_Lat)
                global_position_info.Pixhawk_UTM_x = Pixhawk_UTM_x
                global_position_info.Pixhawk_UTM_y = Pixhawk_UTM_y

                elapsed = time.time() - start
                print elapsed
            else:
                global_position_info.judge = 0
                pass

        self.pub_obstacles_global_position.publish(global_position_info)


    def Calc_camera_UTM(self, yaw_rad, Pixhawk_Lon, Pixhawk_Lat):
        Pixhawk_UTM_x, Pixhawk_UTM_y = pyproj.transform(WGS84, UTM54, Pixhawk_Lon, Pixhawk_Lat)
        yaw_rot = np.array([[math.sin(yaw_rad), math.cos(yaw_rad)],
                            [(-1)*math.cos(yaw_rad), math.sin(yaw_rad)]])

        camera_xy_from_GPS_rot = np.dot(yaw_rot, self.camera_xy_from_GPS.T)
        camera_UTM_x = Pixhawk_UTM_x + camera_xy_from_GPS_rot.item(0)
        camera_UTM_y = Pixhawk_UTM_y + camera_xy_from_GPS_rot.item(1)
        camera_UTM_xy = np.array([camera_UTM_x, camera_UTM_y])

        return camera_UTM_xy


    def Get_depth(self, obstacle_id, center, cv_image_depth_resize):
        integration_depth = 0
        average_depth = None

        image_yx_min = [0, 0]
        image_yx_max = [432, 768]
        count = 0

        #get depth kernel 9*9

        #kernel_size = 9
        #kernel_param = (kernel_size - 1) / 2

        #yx_min = [center[obstacle_id].center_y - kernel_param, center[obstacle_id].center_x - kernel_param]
        #yx_max = [center[obstacle_id].center_y + kernel_param + 1, center[obstacle_id].center_x + kernel_param + 1]


        kernel_size_v = 25
        kernel_size_u = 5

        yx_min = [center[obstacle_id].center_y - kernel_size_v, center[obstacle_id].center_x - kernel_size_u]
        yx_max = [center[obstacle_id].center_y + kernel_size_v, center[obstacle_id].center_x + kernel_size_u]


        for j in range(0,2):
            if yx_min[j] >= image_yx_min[j]:
                pass
            else:
                yx_min[j] = image_yx_min[j]

        for k in range(0,2):
            if yx_max[k] <= image_yx_max[k]:
                pass
            else:
                yx_max[k] = image_yx_max[k]

        for y in range(yx_min[0], yx_max[0]):
            for x in range(yx_min[1], yx_max[1]):
                depth = cv_image_depth_resize.item(y, x)
                if depth != 0:
                    integration_depth = integration_depth + depth
                    count += 1
                else:
                    pass

        if count != 0:
            average_depth = float(integration_depth / count)

        else:
            pass

        return average_depth

    def Convert_camera_to_world(self, obstacle_id, center, depth):
        XYZ3D = np.empty(3)
        XYZ3D[0] = float((center[obstacle_id].center_x - self.cx) * depth) / float(self.fx * 1000)
        XYZ3D[1] = float((center[obstacle_id].center_y - self.cy) * depth) / float(self.fy * 1000)
        XYZ3D[2] = float(depth / 1000)

        return XYZ3D

    def Imu_Camera_rotation(self, obstacle_raw_XYZ3D, roll_rad, pitch_rad):

        imu_roll_rot = np.array([[math.cos(roll_rad), math.sin(roll_rad), 0],
                                 [(-1)*math.sin(roll_rad), math.cos(roll_rad), 0],
                                 [0, 0, 1]])

        imu_pitch_rot = np.array([[1 , 0, 0],
                                  [0, math.cos(pitch_rad), (-1)*math.sin(pitch_rad)],
                                  [0, math.sin(pitch_rad), math.cos(pitch_rad)]])

        camera_pitch_rot = np.array([[1 , 0, 0],
                                   [0, math.cos(self.camera_rad), (-1)*math.sin(self.camera_rad)],
                                   [0, math.sin(self.camera_rad), math.cos(self.camera_rad)]])

        imu_roll_XYZ3D = np.dot(imu_roll_rot, obstacle_raw_XYZ3D.T)
        imu_pitch_XYZ3D = np.dot(imu_pitch_rot, imu_roll_XYZ3D.T)
        camera_pitch_XYZ3D = np.dot(camera_pitch_rot, imu_pitch_XYZ3D.T)

        return camera_pitch_XYZ3D

if __name__ == '__main__':
    WGS84 = pyproj.Proj('+init=EPSG:4326')
    UTM54 = pyproj.Proj('+init=EPSG:32654')
    main()
