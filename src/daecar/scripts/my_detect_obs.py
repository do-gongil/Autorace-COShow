#!/usr/bin/env python
#-*- coding: utf-8 -*-


# =============== pulish 'warning or safe' and 'y' to 'main'====================

import rospy
import math
import time
# from collections import deque
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32, Float32
from obstacle_detector.msg import Obstacles

class LidarReceiver():
    def __init__(self):
        # rospy.loginfo("LiDAR Receiver Object is Created")
        # rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        # rospy.Subscriber("/raw_obstacles", Obstacles, self.lidar_callback)
        rospy.Subscriber("/scan",LaserScan,self.basic_lidar_callback)
        #lidar_msg = LaserScan()

        # rospy.Subscriber("clustering", Obstacles, self.clustering_callback)
        self.warning_pub = rospy.Publisher("lidar_warning", String, queue_size=5)
        self.clear_pub = rospy.Publisher("clear", String, queue_size=5)

        self.object_pub = rospy.Publisher("object_condition", Float32, queue_size=5)
        self.stordy_pub = rospy.Publisher("static_or_dynamic", String, queue_size=5)
        self.leftro_pub = rospy.Publisher("light_obj", Float32, queue_size=5)
        self.light_pub = rospy.Publisher("right_obj", Float32, queue_size=5)
        
        self.blocker_pub = rospy.Publisher("blocker_status", Int32, queue_size=5)

        self.count_flag = 0
        self.flag_flag = 0
        self.count_t1 = 0
        self.x1 = 0
        self.x2 = 0
        self.front_obj = 0
        self.left_obj = 0
        self.light_obj = 0
        self.lightwaringcnt = 0
        self.leftwaringcnt = 0
        
        self.blocker_detected = False
        self.blocker_height = 0
        self.blocker_detection_count = 0
        self.blocker_up_count = 0
        self.BLOCKER_DETECTION_THRESHOLD = 3 #연속 감지 횟수 임계값. (노이즈 방지용, 3번 연속 감지시)
        
        self.rotary_left_pub = rospy.Publisher("rotary_left", Int32, queue_size=5)
        self.rotary_right_pub = rospy.Publisher("rotary_right", Int32, queue_size=5)
        
        self.rotary_left_count = 0
        self.rotary_right_count = 0
        self.ROTARY_DETECTION_THRESHOLD = 3
        
        self.turnnel_right = 0
        self.error = 0

    def basic_lidar_callback(self,_data):
        degrees = [(_data.angle_min + _data.angle_increment*index)*180/math.pi for index, value in enumerate(_data.ranges)]
        
        # 차단기 감지
        blocker_detected_this_scan = False  
        # blocker_up_this_scan = False
        
        # 로터리 장애물 감지
        rotary_left_detected = False
        rotary_right_detected = False

            # # 차단기 감지 영역
            # if (120 <= abs(degrees[index]) <= 180  and 0.3 < _data.ranges[index] < 0.4):  #정면 양쪽(-180°~-175° 또는 162°~180°)

            #  # 차단기 예상 거리 (0.3~1.3m)
            #     blocker_detected_this_scan = True
            #     print("@@@@@@@@@@@@@@@@@2")
            #     self.blocker_height = _data.ranges[index]
                    
            # elif (-180 <= degrees[index] <= -175 or 162 <= degrees[index] < 180 and _data.ranges[index] > 2.0):  # 차단기가 올라간 경우 (2m)
            #     blocker_up_this_scan = True
        
        # for index, value in enumerate(_data.ranges):
        #     if  -180 <= degrees[index] <= -176  and 0 < _data.ranges[index] < 1.4:   
        #         self.front_obj = 1
        #     elif  162 <= degrees[index] < 180 and 0 < _data.ranges[index] < 1.4:   
        #         self.front_obj = 1
        #     elif 100 < degrees[index] <= 162 and  0 < _data.ranges[index] < 2.0:
        #         self.light_obj = 1
        #     elif -176 < degrees[index] < -140 and  0 < _data.ranges[index] < 2.0:
        #         self.left_obj = 1
        for index, value in enumerate(_data.ranges):
            if -170 < degrees[index] < -150 and  0 < _data.ranges[index] < 0.80:
                self.leftwaringcnt +=1
                self.turnnel_left = _data.ranges[index]
                # print("leftvalue",_data.ranges[index])
            # if -180 < degrees[index] < -130 and  0 < _data.ranges[index] < 0.90:
                # self.leftwaringcnt +=1
                # self.turnnel_left = _data.ranges[index]
                # print("leftvalue",_data.ranges[index])    
                
            elif 150 < degrees[index] < 170 and  0 < _data.ranges[index] < 0.80:
                self.lightwaringcnt +=1
                # print("leftvalue",_data.ranges[index])
                self.turnnel_right = _data.ranges[index]
                # print("right",self.turnnel_right)
            
            # 로터리 왼쪽
            if -179 <= degrees[index] <= -140 and 0 < _data.ranges[index] < 0.8:
                # print("@@@@@@@@@@@@@@@@@@@@@@@")
                rotary_left_detected = True
            
            #로터리 오른쪽
            elif (-180 <= degrees[index] <= -155 or 170 <= degrees[index] <= 180)  and 0 < _data.ranges[index] < 0.50:
                rotary_right_detected = True
                
            # 차단기 감지 영역
            # elif ((-180 <= degrees[index] <= -162.7) or (175 <= degrees[index] <= 180)) and (0 < _data.ranges[index] < 0.6) :  #정면 양쪽(-180 °~-175° 또는 162°~180°)
                # 차단기 예상 거리 (0.3~1.3m)
                # blocker_detected_this_scan = True
                # print("###### bloack")
                # self.blocker_height = _data.ranges[index]

            # else:  # 차단기가 올라간 경우 (2m)
            #     blocker_up_this_scan = True

        #     else:
        #         pass

        # 왼쪽 로터리 상태 업데이트
        if rotary_left_detected:
            self.rotary_left_count += 1
        else:
            self.rotary_left_count = 0

        # 오른쪽 로터리 상태 업데이트
        if rotary_right_detected:
            self.rotary_right_count += 1
        else:
            self.rotary_right_count = 0
            
        # # 차단기 상태 업데이트
        # if blocker_detected_this_scan:
        #     self.blocker_detection_count += 1
        #     self.blocker_up_count = 0
        # elif blocker_up_this_scan:
        #     self.blocker_up_count += 1
        #     self.blocker_detection_count = 0
        # else:
        #     self.blocker_detection_count = 0
        #     self.blocker_up_count = 0

        # 로터리(장애물) 상태 발행
        if self.rotary_left_count >= self.ROTARY_DETECTION_THRESHOLD:
            self.rotary_left_pub.publish(1)  # 왼쪽 로터리 감지
        else:
            self.rotary_left_pub.publish(0)  # 왼쪽 로터리 미감지

        if self.rotary_right_count >= self.ROTARY_DETECTION_THRESHOLD:
            self.rotary_right_pub.publish(1)  # 오른쪽 로터리 감지
        else:
            self.rotary_right_pub.publish(0)  # 오른쪽 로터리 미감지
            
        # # 차단기 상태 발행
        # if self.blocker_detection_count >= self.BLOCKER_DETECTION_THRESHOLD:
        #     self.blocker_pub.publish(1)  # 차단기 감지됨
        # # elif self.blocker_up_count >= self.BLOCKER_DETECTION_THRESHOLD:
        #     # self.blocker_pub.publish(2)  # 차단기가 올라감
        # else:
        #     self.blocker_pub.publish(0)  # 차단기 없음
            
        
                    
        if self.lightwaringcnt >= 3 and self.leftwaringcnt >=3:
            self.error = (self.turnnel_left - self.turnnel_right)/2

            turnnel_steer = 2 * (self.error + 0.1)
            # turnnel_steer = 2 * (self.error +0.05)
            print("turnnel_steer:",turnnel_steer)
            self.light_pub.publish(turnnel_steer)
            self.lightwaringcnt = 0
            self.leftwaringcnt = 0 

        else:
            self.light_pub.publish(100000)
            # self.lightwaringcnt = 0
            
 

        #     self.leftro_pub.publish(1)
            
        # else:
        #     self.left_pub.publish(0)
        #     self.leftwaringcnt = 0 


        #rospy.loginfo(self.front_obj)
        if self.front_obj == 1 and self.left_obj == 1 and self.light_obj == 1:
            self.stordy_pub.publish("dynamic")
            #rospy.loginfo("dynamic")
            self.left_obj = 0
            self.front_obj = 0
            self.light_obj = 0 
        else :
            self.stordy_pub.publish("none")

            self.left_obj = 0
            self.front_obj = 0
            self.light_obj = 0 
         
 
    # def lidar_callback(self, _data):

    #     #rospy.logwarn("lidar callback")
    #     #rospy.loginfo("x:{}".format(_data.circles[0].center.x))

    #     # ROI
    #     left_y = -0.20
    #     right_y = 0.33
    #     front_x = -1.5
    #     back_x = 0
    #     WARNING_CNT = 1
        
    #     # warning ROI 
    #     Wleft_y = -0.50
    #     Wright_y = 0.50
    #     Wfront_x = -1.55
    #     Wback_x = -0.3

    #     self.point_cnt = 0
    #     self.dynamic_cnt = 0
    #     self.Wpoint_cnt = 0
    #     # 동적장애물 왼쪽 기둥 인식 code
        

    #     for i in _data.circles :
    #         if (-0.07 < i.center.y < 0.11 and -0.35 < i.center.x < -0.2) or (left_y < i.center.y < right_y and -1.20 < i.center.x < -0.35): #if left_y < i.center.y < right_y and -1.20 < i.center.x < -0.2:
    #             #rospy.loginfo("1")
    #             self.point_cnt += 1            # if self.flag_flag == 0 :
                    
    #         if self.point_cnt >= WARNING_CNT:
    #             self.warning_pub.publish("WARNING")
                
    #         else:
    #             self.warning_pub.publish("safe")
    #             self.point_cnt = 0

def run():
    rospy.init_node("lidar_example")
    new_class = LidarReceiver()
    rospy.spin()


if __name__ == '__main__':
    run()