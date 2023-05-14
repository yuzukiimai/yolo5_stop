#!/usr/bin/env python3

import numpy as np
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import os
import sys
import copy
from std_msgs.msg import Float32

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_under_sub = rospy.Subscriber("/camera_under/rgb/image_raw", Image, self.callback_under_camera)
        self.cv_under_image = np.zeros((480,640,3), np.uint8)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_pub = rospy.Publisher('/pie_vel', Float32, queue_size=1)
        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.cv_left_image = np.zeros((480,640,3), np.uint8)
        self.cv_right_image = np.zeros((480,640,3), np.uint8)
        self.is_started = False
        self.img_number = 0

    def callback_under_camera(self, data):
        try:
            self.cv_under_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)   

    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def loop(self):
        if self.cv_under_image.size != 640 * 480 * 3:
            return
        if self.vel.linear.x != 0:
            self.is_started = True
        if self.is_started == False:
            return

        self.img_number += 1

        img_under = self.cv_under_image

        cv2.imwrite('/home/yuzuki/Pictures/collect/' + str(self.img_number) + '.jpg', img_under)

        if self.episode == 5700:
            os.system('killall roslaunch')
            sys.exit()

        self.episode += 1
        self.vel.linear.x = 0.2
        self.vel.angular.z = self.action
        self.nav_pub.publish(self.vel)
        self.vel_pub.publish(self.vel.angular.z)

        temp = copy.deepcopy(img_under)
        cv2.imshow("Under Image", temp)
        cv2.waitKey(1)

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
