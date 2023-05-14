#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge, CvBridgeError
import copy

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_pub = rospy.Publisher('/pie_vel', Float32, queue_size=1)
        self.detect_sub = rospy.Subscriber("/detect", Bool, self.callback_detect)
        self.image_under_sub = rospy.Subscriber("/camera_under/rgb/image_raw", Image, self.callback_under_camera)
        self.bridge = CvBridge()
        self.cv_under_image = np.zeros((480,640,3), np.uint8)
        self.vel = Twist()
        self.is_started = False
        self.detect = False
        self.stop_flg = False


    def callback_under_camera(self, data):
        try:
            self.cv_under_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def callback_vel(self, data):
        self.vel = data


    def callback_detect(self, data):
        self.detect = data.data
        if self.detect:
            self.stop_flg = True
            
        else:
            self.stop_flg = False


    def loop(self):
        if self.cv_under_image.size != 640 * 480 * 3:
            return

        if self.vel.linear.x != 0:
            self.is_started = True

        if self.is_started == False:
            return

        img_under = self.cv_under_image

        if self.stop_flg:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.nav_pub.publish(self.vel)
            self.vel_pub.publish(self.vel.angular.z)

        else:
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
