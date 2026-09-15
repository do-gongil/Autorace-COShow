#!/usr/bin/env python
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from time import sleep, time
from SlideWindow import cameraReceiber

from std_msgs.msg import Int32, String, Float32

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from fiducial_msgs.msg import Fiducial, FiducialArray


class Controller():
    def __init__(self):
        self.slidewindow = cameraReceiber()
        self.bridge = CvBridge()
        self.initialized = False
        self.slide_img = None 
        self.slide_x_location = 0
        self.middle_lane = 320
        self.error_lane = 0
        self.nothing_flag = False
        self.center_fit_x = 0
        self.speed_lane = 0.4
        self.acenter_fit_x = 320
        
        # for slow down mission(child protection zone)
        self.slow_flag = 0
        self.slow_down_flag = 0
        self.slow_t1 = 0.0
        self.sign_data = 0
        self.child_cnt = 0

        self.drive_pub = rospy.Publisher("/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)

        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.camera_callback)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)

        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        self.speed_stop = 0.0

    def camera_callback(self,_data):
        img = self.bridge.imgmsg_to_cv2(_data)
        kernel_size = 5

        blur_img = cv2.GaussianBlur(img,(kernel_size, kernel_size), 0)
        
        _, L, _ = cv2.split(cv2.cvtColor(blur_img, cv2.COLOR_BGR2HLS))
        _, lane = cv2.threshold(L, 160, 255, cv2.THRESH_BINARY)
        
        #cv2.imshow("original", img) 
        #cv2.waitKey(1)
        #lower = np.uint8([0, 150, 0]) # 최소 흰색 범위 정의
        #upper = np.uint8([255, 255, 255]) # 최대 흰색 범위 정의
        #white_mask = cv2.inRange(hls, lower, upper)

        #cv2.imshow("asd", lane)
     

        #blend_color = self.slidewindow.detect_color(img, hsv)
        #cv2.imshow("blur_img", blur_img)       
        blend_line = self.slidewindow.img_warp(lane)

        cv2.imshow("blend_line", blend_line )

        if self.nothing_flag == False:
            self.slidewindow.detect_nothing()
            self.nothing_flag = True
        (
            sliding_window_img,
            left,
            right,
            center,
            left_x,
            left_y,
            right_x,
            right_y,self.center_fit_x,self.acenter_fit_x
        ) = self.slidewindow.window_search(blend_line)
        
        cv2.imshow("Daelimcar_camera", sliding_window_img )
        cv2.waitKey(1)

    def lane_drive(self):

        if self.slow_down_flag == 1:
            if self.sign_data == 3:
                rospy.loginfo(" ===============   SLOW DETECTED, WAIT!!!! ============")
                self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
                publishing_data = AckermannDriveStamped()
                publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
                publishing_data.header.frame_id = "base_link"
                publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
                publishing_data.drive.speed = self.speed_lane
                self.drive_pub.publish(publishing_data) #  하는 부분 

            elif self.sign_data == 0 :
                rospy.loginfo("************* SLOW DOWN *****************")        
                self.child_cnt = 0        
                if self.slow_flag == 0:
                    self.slow_t1 = rospy.get_time()
                    self.slow_flag = 1
                t2 = rospy.get_time()
                #rospy.loginfo("t1 :{}, t2 : {}".format(self.slow_t1, t2))
                # the car should stop for more than 14 seconds
                while t2-self.slow_t1 <= 15 :
                    rospy.loginfo("************* SLOW DOWN *****************")
                    #rospy.loginfo("slow_down time{}, {}".format(self.slow_t1,t2))
                    self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
                    publishing_data = AckermannDriveStamped()
                    publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
                    publishing_data.header.frame_id = "base_link"
                    publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
                    publishing_data.drive.speed = self.speed_slow
                    self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분
                    t2 = rospy.get_time()
                self.slow_down_flag = 0
                self.slow_flag = 0
                # self.y_list = []

        self.error_lane = 320 - self.acenter_fit_x
        rospy.loginfo(self.error_lane)
        #self.follow_lane()
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = self.error_lane * 0.0054 # 대상꺼는 0.003 error 정도에 따라서 조향 
        publishing_data.drive.speed = self.speed_lane #main_class_run이 데이터를 보낼 때의 시점      
        self.drive_pub.publish(publishing_data)

    def stop(self): 
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = self.speed_stop
        self.drive_pub.publish(publishing_data)

    def child_sign_callback(self, _data):
        try :
            # : {}".format(_data.data))

            if _data.data == 3:
                self.child_cnt += 1
                rospy.loginfo("DEFAULT DRIVE!!!!!!!!!")
                if self.child_cnt >=20 :
                    self.sign_data = _data.data
                    self.slow_down_flag = 1
                    self.child_cnt = 0
            else :
                self.sign_data = 0
                # self.slow_down_flag = 0
            #rospy.loginfo(" sign data_callback  : {}".format(self.sign_data))

            #if _data.data == 3:
            #    self.slow_down_flag = 1
                
        except :
            pass


    def timer_callback(self, _event):
        try :
            self.lane_drive()
        except :
            pass

def nothing(x):
    pass

def run():

    rospy.init_node("maincontroller")
    controller = Controller()
    rospy.Timer(rospy.Duration(1.0/30.0), controller.timer_callback) 
    rospy.spin()

if __name__ == "__main__":
    run()