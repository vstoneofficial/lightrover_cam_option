#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、カメラで特定の色を検出するノードです。

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()
speed = Twist()
pose = Pose()
AREA_SIZE_LIMIT = 0.005

def callback(ros_image):
    global bridge
    try:
        print('Get image')
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    input_image = np.array(frame, dtype=np.uint8)

    color_detect(input_image)
    cv2.waitKey(1)

def color_detect(image):
    global pose
    #画像の大きさを取得
    h,w,_ = image.shape
    #画像をBGRからHSVに変換
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #hsvの認識範囲指定
    lower_color = np.array([168,100,100])
    upper_color = np.array([179,255,255])
    mask = cv2.inRange(hsv, lower_color, upper_color)  

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        approx = cv2.convexHull(contour)
        rect = cv2.boundingRect(approx)
        rects.append(np.array(rect))

    size = 0.0
    x = 0.0
    
    if len(rects) > 0:
        rect = max(rects, key=(lambda x:x[2] * x[3]))
        #検出範囲に赤枠を描写
        cv2.rectangle(image, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
        cv2.imshow('image',image)
    
        #検出範囲の面積を算出
        area = np.array(list(map(cv2.contourArea,contours)))
        size = np.max(area)/(h*w)

        if len(area) > 0 and size >= AREA_SIZE_LIMIT:
            max_idx = np.argmax(area)
            max_area = area[max_idx]
            result = cv2.moments(contours[max_idx])
            #検出範囲の左右中心を算出
            x = float(w/2-int(result["m10"]/result["m00"]))/(w/2)
            print([x,size])
    
    pose.position.x = x
    pose.position.z = size


def color_detector():
    global pose, color_img
    rospy.init_node("color_detect", anonymous=True)
    print('Start color detect')
    sub = rospy.Subscriber("image", Image, callback, queue_size=1)
    pub = rospy.Publisher('color_position',Pose, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pub.publish(pose)
        rate.sleep()
    
if __name__ == '__main__':
    color_detector()
    cv2.destroyAllWindows()
