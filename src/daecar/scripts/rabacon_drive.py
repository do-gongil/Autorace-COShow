#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from time import sleep
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, Float32


class ClusterLidar :

    def __init__(self) :
        rospy.Subscriber("/raw_obstacles", Obstacles, self.rabacon)
        self.rabacon_pub = rospy.Publisher("rabacon_drive", Float32, queue_size = 5)
        self.angle = 0.0 

    # select close rabacon
    def rabacon(self, _data)  :
        left_rabacon = []
        right_rabacon = []
        for i in _data.circles :
            if -0.3742 < i.center.x < 0: # 정면 인식 범위 1.5M  # 0.4
                # print("@#!@!@$@!$!@$!@$!@$!@$!@$@")
                if -1.2 < i.center.y < 0 : # -1 왼쪽 범위 1m ~ 오른쪽 0.2m #-1.4
                   left_rabacon.append(i)
                elif 0 < i.center.y < 1.2 : # 1 오른쪽 범위 1m ~ 왼쪽 0.2m #1.2
                    right_rabacon.append(i)   
        # print("left_rabacon", left_rabacon)0
        # print("right_rabacon", right_rabacon)
        # print("len(right_rabacon):",len(left_rabacon))
        if len(left_rabacon) >= 1 and len(right_rabacon) >= 1:
            left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
            right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
            raba = ((left_close_rabacon.center.y) * 1.3 + (right_close_rabacon.center.y) * 1.2)
            # print("raba",raba)
            self.rabacon_pub.publish(raba)

            #rospy.loginfo(right_rabacon[0])
        elif len(left_rabacon) == 0 and len(right_rabacon) >= 1:
        # # elif len(left_rabacon) < len(right_rabacon):

            self.rabacon_pub.publish(-0.27)
        elif len(left_rabacon) >= 1 and len(right_rabacon) == 0:
            self.rabacon_pub.publish(0.27)
        else :
            self.rabacon_pub.publish(1000.0)

    # def cal_angle(self, left_rabacons, right_rabacons) :
    #     left_angle = abs(math.atan(left_rabacons[0].center.y/left_rabacons[0].center.x))
    #     right_angle =  abs(math.atan(right_rabacons[0].center.y/right_rabacons[0].center.x))
    #     rospy.loginfo("left angle = {} right angle = {}".format(left_angle, right_angle))
    #     return (left_angle - right_angle)*0.5


    # def sel_close_rabacon(self, rabacons) :
    #     left_rabacon = []
    #     right_rabacon = []
    #     raba = 0
    #     for circle in rabacons :
    #         if circle.center.y < 0 :
    #             right_rabacon.append(circle)
    #         else :
    #             left_rabacon.append(circle)
    #     left_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)
    #     right_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)
    #     if len(left_rabacon) != 0 and len(right_rabacon) != 0 :
    #         raba = (left_rabacon[0].center.y + right_rabacon[0].center.y)/2
    #     return raba

def run() :
    rospy.init_node("rabacon_drive")
    cluster = ClusterLidar()
    rospy.spin()

if __name__=='__main__' :
    run()
