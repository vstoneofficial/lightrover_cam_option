#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーをカメラで動かすためのノードです。

import rospy
from geometry_msgs.msg import Twist, Pose

speed = Twist()
pose = Pose()
X_TH = 0.1
Z_H_TH = 0.4
Z_L_TH = 0.01

def call_back(data):
        global speed, X_TH, Z_TH

        #検出された色の大きさが一定範囲内であれば前進する
        if data.position.z < Z_H_TH and data.position.z > Z_L_TH:
                speed.linear.x = (Z_H_TH-data.position.z)*0.2
        else:
                speed.linear.x = 0.0

        #指定範囲より外にいれば検出された方向を向く
        if abs(data.position.x) > X_TH:
                speed.angular.z = data.position.x*0.8
        else:
                speed.angular.z = 0.0
                

        print([speed.linear.x,speed.angular.z])

def rover_cam_controller():
        global speed, pose

        rospy.init_node('rover_cam_controller', anonymous=True)

        pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

        rate = rospy.Rate(20)

        rospy.Subscriber('pose',Pose,call_back)

        while not rospy.is_shutdown():
                pub.publish(speed)
                rate.sleep()

if __name__ == '__main__':
        rover_cam_controller()
