#!/usr/bin/env python
# -*- coding: utf-8 -*-

from this import d
import rospy
from time import sleep, time
import statistics
from collections import deque

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image, CompressedImage
from fiducial_msgs.msg import Fiducial, FiducialArray
from obstacle_detector.msg import Obstacles

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from SlideWindow import cameraReceiber
from rabacon_drive import ClusterLidar


class Controller:

    def __init__(self):
        self.img = []
        self.now_line = "LEFT"
        self.slidewindow = cameraReceiber()
        self.bridge = CvBridge()
        self.rabacon_mission = ClusterLidar()
        # self.decide_obstacle = DecideObstacle()

        # initialization
        # competition day
        self.current_lane = "LEFT"  # which lane
        self.is_safe = True  # assume safe in the beginning
        self.initialized = False  # lane callback
        # self.speed_lane = 0.7 # default speed
        self.speed_lane = 1 * (0.4)
        self.speed_rabacon = 0.34#0.25  # rabacon speed
        self.speed_turn = 0.3  # change to left or right speed
        self.speed_slow = 0.33  # slow down speed(child protection zone)
        self.speed_stop = 0.0  # stop

        #####################
        # self.stoplinecnt = 4  # 0
        self.initialized = False
        self.slide_img = None
        self.slide_x_location = 0
        self.middle_lane = 160
        self.error_lane = 0
        self.nothing_flag = False
        self.center_fit_x = 0
        self.acenter_fit_x = 160
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

        #######################################

        # slide window return variable initialization
        self.current_lane_window = ""

        # for rabacon mission
        self.rabacon_mission_flag = False

        # for slow down mission(child protection zone)
        self.slow_flag = 0
        self.slow_down_flag = 0
        self.slow_t1 = 0.0
        self.sign_data = 0
        self.child_cnt = 0

        # for static obstacle
        self.static_flag = 0
        self.turn_left_flag = 0
        self.turn_right_flag = 0

        # for dynamic obstacle
        self.dynamic_flag = 0
        self.dynamic_flag2 = 0

        # for obstacle type detection
        # self.stop_cnt = 0

        self.obstacle_kind = "None"
        self.obstacle_img = []

        self.red_cnt = 0  # rabacon
        self.green_cnt = 0  # dynamic obstacle
        self.black_cnt = 0  # static obstacle

        self.y_list = []
        self.y_list_sort = []
        self.dynamic_obs_cnt = 0
        self.static_cnt = 0
        self.initstopcnt = False
        self.moter_msg = AckermannDriveStamped()

        self.mode_endtime = 0
        self.mode_change = 1
        self.moter_msg.header.frame_id = "base_link"
        # ======Slow_stop_line=======
        self.slow_mode = 1
        self.init_cnt = False
        self.stop_line_cnt = 0
        self.t4 = 0
        self.stop_min_pix = np.int32(500)  # ROI 영역의 80% 이상이면 감지 # 처음 2000개임
        self.sinho_park_pix = np.int32(1800)
        self.m = 0
        self.stop_line_cnt = 0
        self.change_slow_drivemode = False
        # ============================
        # =======dybanuc=============
        self.dyt = rospy.get_time()
        self.nowtime = rospy.get_time()
        self.dohyun = 1

        # 차단기 미션용
        self.blocker_detected = 0  # False
        self.blocker_up = 0
        self.blocker_wait_time = 4.0  # 대기시간

        rospy.Subscriber("blocker_status", Int32, self.blocker_callback)
        
        # 로터리(장애물) 미션용
        self.rotary_left_detected = 0
        self.rotary_right_detected = 0
        
        rospy.Subscriber("rotary_left", Int32, self.rotary_left_callback)
        rospy.Subscriber("rotary_right", Int32, self.rotary_right_callback)
        
        ############
        self.parking_mode = 1
        self.park_start_flag = 0

        # which lane
        self.drive_mode =  1#2  # 1 = right lane, 2 = left lane

        # ==============================
        self.current_cnt = 0
        self.change_flag = 0
        self.initright_time = False
        self.leftcurrent_cnt = 4  # 0
        self.rightcurrent_cnt = 0
        self.blend_slow_img = []
        rospy.Timer(rospy.Duration(1.0 / 60.0), self.timer_callback)

        self.drive_pub = rospy.Publisher(
            "high_level/ackermann_cmd_mux/input/nav_0",
            AckermannDriveStamped,
            queue_size=1,
        )

        rospy.Subscriber(
            "/usb_cam/image_rect_color/compressed",
            CompressedImage,
            self.camera_callback,
        )
        # rospy.Subscriber("obstacle_mission", String, self.warning_callback)
        rospy.Subscriber(
            "lidar_warning", String, self.warning_callback
        )  # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        # rospy.Subscriber("object_condition", Float32, self.object_callback)
        rospy.Subscriber("static_or_dynamic", String, self.st_or_dy_callback)

        # rospy.Subscriber("clear", String, self.st_or_dy_clear)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)
        rospy.Subscriber("rabacon_drive", Float32, self.rabacon_callback)
        rospy.Subscriber("light_obj", Float32, self.lightobj)
        # rospy.Subscriber("right_obj", Float32, self.leftobj)
        rospy.Subscriber("right_obj", Float32, self.turnnel_callback)  ## 터널로 쓸꺼임
        ##터널
        self.turnnel_mission = 0
        self.speed_turnnel = 0.2
        self.turnnel_endtime = 0  # rospy.get_time()
        self.lys = 1
        self.line_right_time = 0
        self.rabastart_time = 0#rospy.get_time()
        self.rabastart = 1
        # rospy.Subscriber("obstacles", Obstacles, self.rabacon_callback)
        self.SOD_list = []
        self.dynamic_obj = 0
        self.dynamic_cnt = 0
        self.dynamic_obsclear = 0
        self.warninglightobj = 0
        self.warningleftobj = 0
        self.cnt = 0
        self.t55 = 0
        self.red_area_ratio = 0
        self.speed_red_slow = 0.2
        self.stoplinecheck = 0  # 4#0  # 4: 좌측차선보고 >> 로타리  // 5: 로터리 주행 // 6:신호등 앞 // 7: 신호등 후 우회전 // 8: 눈감고 직진(마지막)
        self.sinho_park_check = 0
        self.istf = 0
        self.ste = 0
        self.ste_2 = 0

        rospy.Subscriber("blocker_status", Int32, self.blocker_callback)
        ### 정지선 2개 읽고 15초 후 로터리 차단막 안보는 곳 관련 변수
        self.rotter_blocker_flag = 1
        self.jungji_end_time = 0
        self.jungji_time = 0
        self.jungji = 1
        #장애들
        self.l_jange = 0
        self.r_jange = 0

        self.turnnel_switch = 0
        self.rababa_mission = 1
        self.rotblock_start = 0
        ############

    # def st_or_dy_clear(self,_data):
    #    if _data.data == "CLEAR" :
    #        self.dynamic_obsclear += 1
    #        if self.dynamic_obsclear >= 20 :
    #            self.dynamic_obsclear = 0
    #            self.dynamic_obj = 0
    #            self.SOD_list = []
    def lightobj(self, _data): # leftro 라이다에서 받아온거
        if _data.data == 1:
            self.warninglightobj = 1
        else:
            self.warninglightobj = 0

    def leftobj(self, _data):
        if _data.data == 1:
            self.warningleftobj = 1
        else:
            self.warningleftobj = 0

    # 로터리 (장애물) 콜백 함수
    def rotary_left_callback(self, _data):
        self.rotary_left_detected = _data.data

    def rotary_right_callback(self, _data):
        self.rotary_right_detected = _data.data
        
    # 차단기 콜백 함수
    def blocker_callback(self, _data):
        if _data.data == 1:  # 차단기 감지됨
            self.blocker_detected = 1123123123123213
            self.blocker_up = 0

        elif _data.data == 2:  # 차단기 올라감
            self.blocker_up = 1

        else:  # 차단기 없음
            self.blocker_detected = 0
            self.blocker_up = 1

    def st_or_dy_callback(self, _data):
        # if abs(_data.data) > 0.008:
        # self.SOD_list.append(_data.data)

        if _data.data == "dynamic":
            self.dynamic_obj = 1
        else:
            self.cnt += 1
            if self.cnt >= 1:
                self.dynamic_obj = 0

        # rospy.loginfo(self.dynamic_obj)
        # if len(self.SOD_list) >= 2:
        #    meanval= statistics.mean(self.SOD_list)
        #    #rospy.loginfo()
        #    for i in self.SOD_list:
        #        if i - meanval > 0.8:
        #            self.dynamic_obj = 1
        #            self.dynamic_obsclear = 0

    #
    # self.dynamic_cnt += 1
    # if self.dynamic_cnt >= 2:
    #    self.dynamic_obj = 1
    #    self.dynamic_cnt = 0
    # if len(self.SOD_list) >= 21 : #21
    #    del self.SOD_list[0]

    def object_callback(self, _data):
        if self.dynamic_flag != 1 or len(self.y_list) <= 19:
            self.y_list.append(_data.data)
            if len(self.y_list) >= 21:  # 21
                del self.y_list[0]
        else:
            rospy.logwarn("Unknown warning state!")

    # 장애물 감지
    def warning_callback(self, _data):

        # if self.rabacon_mission_flag == True :
        #     self.rabacon_mission_flag = True
        if _data.data == "safe":
            self.is_safe = True
            self.y_list = []
            if self.dynamic_flag == 1:
                self.dynamic_obs_cnt += 1
                if self.dynamic_obs_cnt >= 19 and len(self.y_list) <= 5:  # 50
                    self.dynamic_flag = 0
                    self.dynamic_obs_cnt = 0
            self.static_flag = 0
        elif _data.data == "WARNING":
            self.is_safe = False
            # rospy.loginfo("WARNING!")
        else:
            pass

    def rabacon_callback(self, _data):
        

        self.rabacon_mission_flag = _data.data
        # print("self.rabacvon",self.rabacon_mission_flag)
        if self.rabacon_mission_flag < 10.0:
            self.rabacon_mission = 1
        else:
            self.rabacon_mission = 0
            # self.change_line_right(0.3)

    def camera_callback(self, _data):
        self.img = self.bridge.compressed_imgmsg_to_cv2(_data)
        img = cv2.resize(self.img, (320, 240))
        kernel_size = 5
        # cv2.imshow("img", img)
        blur_img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
        hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
        slow_roi = hsv_img[195:215, 110:210]
        # _, L, _ = cv2.split(cv2.cvtColor(blur_img, cv2.COLOR_BGR2HLS))
        yellow_lower = np.array([10, 90, 17])  # 색상,채도,명도
        yellow_upper = np.array([40, 255, 173])
        yellow_range = cv2.inRange(
            hsv_img, yellow_lower, yellow_upper
        )  # mask or filter

        # white_lower = np.array([0, 0, 108])
        # white_upper = np.array([179, 255, 255])
        white_lower = np.array([0, 0, 140]) ## 원래보는 값. 대회떄 1개 읽음.
        white_upper = np.array([179, 100, 215])

        white_range = cv2.inRange(hsv_img, white_lower, white_upper)  # mask or filter

        # blue_lower = (0, 180, 55)
        # blue_upper = (20, 255, 200)
        # blue_range = cv2.inRange(hsv_img, blue_lower, blue_upper)# mask or filter

        red_lower1 = np.array([0, 54, 25])
        red_upper1 = np.array([11, 255, 255])
        lower_red = np.array([160, 80, 30])
        upper_red = np.array([179, 255, 255])
        red_mask1 = cv2.inRange(slow_roi, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(slow_roi, lower_red, upper_red)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        # red_mask_warp = self.slidewindow.img_warp(red_mask) # 320 240 빨간거만 필터링 한거 이미줘읖

        red_pixels = cv2.countNonZero(red_mask)

        img_slow = self.slidewindow.img_warp(img)  # 320 240 이미지 워프

        total_pixels = img_slow.shape[0] * img_slow.shape[1]
        self.red_area_ratio = red_pixels / total_pixels
        # cv2.imshow("red_mask_warp", red_mask)
        # print("redddddddddddddd", self.red_area_ratio)
        # 결과 시각화를 위한 디버깅 창 추가
        # cv2.imshow("Yellow Detection", yellow_range)
        # cv2.imshow("White Detection", white_range)
        combined_img = cv2.bitwise_or(
            yellow_range, yellow_range
        )  # hsv_y_range 와 hsv_wh_range 이미지를 합침
        filtered_img = cv2.bitwise_and(img, img, mask=combined_img)

        blend_line = self.slidewindow.img_warp(
            filtered_img
        )  # filtered_img는 기본이미지에 노란색과 하얀선을 합친이미지 == blend_line
        grayed_img = cv2.cvtColor(
            blend_line, cv2.COLOR_BGR2GRAY
        )  # blend_line은 합친이미지에서 2차원 평면으로 차선을 편 이미지 >> 이를 grayscale 로 전환한 이미지가 grayed_img
        threshold_value = 100 # 임계값
        _, binary_img = cv2.threshold(grayed_img, threshold_value, 255, cv2.THRESH_BINARY)
        white_line = self.slidewindow.img_warp(white_range)
        self.stop_img = white_line[200:220, 100:230].copy()
        # cv2.imshow("stop_img", self.stop_img)
        stop_cnt = self.stopline_ROI(self.stop_img)
        self.stop_mod = 1 if stop_cnt == 8 else 0

        # lane = cv2.adaptiveThreshold(
        # grayed_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 7
        # )
        blend_line = self.slidewindow.img_warp(binary_img)
        self.blend_slow_img = white_line.copy()
        # cv2.imshow("grey_img", grayed_img)

        if self.nothing_flag == False:
            self.slidewindow.detect_nothing()
            print("@@@@@@@@@@@@@@@@@@@@#!#!")
            # self.change_line_left()
            # sleep(0.7)
        self.nothing_flag = True
        (sliding_window_img, self.acenter_fit_x, self.current_lane) = (
            self.slidewindow.window_search(blend_line, self.drive_mode)
        )
        # stopline_detect_img = self.blend_slow_img[195:215, 115:245]
        # cv2.imshow("stopline_detect_img", stopline_detect_img)
        # cv2.imshow("Daelimcar_camera", sliding_window_img)
        cv2.waitKey(1)
        # self.change_linecnt()
        ## 주차하는 곳

    def parking_start(self):
        print("@@@@@@@@@@@@@@@@@@@@@@@@")
        self.moter_msg.header.stamp = rospy.Time.now()
        self.moter_msg.drive.speed = 0.2
        self.moter_msg.drive.steering_angle = -0.067
        self.drive_pub.publish(self.moter_msg)

    def parking_back_1(self):
        self.moter_msg.header.stamp = rospy.Time.now()
        self.moter_msg.drive.speed = -0.2
        self.moter_msg.drive.steering_angle = 0.93
        self.drive_pub.publish(self.moter_msg)

    def parking_back_2(self):
        self.moter_msg.header.stamp = rospy.Time.now()
        self.moter_msg.drive.speed = -0.2
        self.moter_msg.drive.steering_angle = -1.0
        self.drive_pub.publish(self.moter_msg)

    ## 주차하는 곳#############3#############3#############3#############3
    def parking_out_1(self):
        self.moter_msg.header.stamp = rospy.Time.now()
        self.moter_msg.drive.speed = 0.2
        self.moter_msg.drive.steering_angle = -1.0
        self.drive_pub.publish(self.moter_msg)

    ## 주차하는 곳#############3#############3#############3#############3

    def parking_out_2(self):
        self.moter_msg.header.stamp = rospy.Time.now()
        self.moter_msg.drive.speed = 0.2
        self.moter_msg.drive.steering_angle = 1.0
        self.drive_pub.publish(self.moter_msg)

    def park_korean_stop(self):
        self.moter_msg.header.stamp = rospy.Time.now()
        self.moter_msg.drive.speed = 0.0
        self.moter_msg.drive.steering_angle = -1.0
        self.drive_pub.publish(self.moter_msg)

    ## 주차하는 곳#############3#############3#############3#############3

    def handle_stopline(self):
        try:
            # stop_pixels = self.stopline_ROI() # 원랴ㅐ는 레인픽셀 들어오는 곳
            # print(len(stop_pixels))
            if (
                self.sinho_park_check == 1
            ):  # Changed threshold to 2080 self.stoplinecheck is not None and
                self.drive(0)  # Stop the vehicle
                rospy.loginfo("Stopline detected - Stopping")
                sleep(6)
                rospy.loginfo("Starting again")
                self.dohyun = 0
                return True
            return False
        except:
            return False

    def pid_control(self, cte):
        self.kp = 0.25
        self.ki = 0.0000
        self.kd = 0.025 * (0.25)
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error

    def timer_callback(self, _event):
        try:
            self.follow_lane()
        except:
            pass

    def turnnel_callback(self, _data):

        self.turnnel_mission_flag = _data.data
        # print("self.rabacvon",self.rabacon_mission_flag)
        if self.turnnel_mission_flag < 10.0:
            self.turnnel_mission = 1
        else:
            self.turnnel_mission = 0
    
    def drive(self, speed):

        self.error_lane = 360 - self.acenter_fit_x
        pid_angle = self.pid_control(self.error_lane)
        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.steering_angle = (
            pid_angle * 0.003
        )  # self.error_lane * 0.003 # 대상꺼는 0.003 error 정도에 따라서 조향
        self.moter_msg.drive.speed = speed  # main_class_run이 데이터를 보낼 때의 시점
        self.drive_pub.publish(self.moter_msg)  #  하는 부분

    def stopline_ROI(self, img):
        ## 겨울꺼 추가
        if self.istf == 1:
            self.ste = rospy.get_time()  # 과거시간 측정
            self.ste_2 = rospy.get_time()
            self.istf = 0
        stt = rospy.get_time()  # 현재시간 측정
        lane_pixel = cv2.countNonZero(img)
        ###
        # stopline_detect_img = self.blend_slow_img[195:215, 115:245]
        # print("*****")
        # lane_pixel = cv2.findNonZero(stopline_detect_img)
        if stt - self.ste >= 6.5 and lane_pixel > self.stop_min_pix:
            self.stoplinecheck += 1
            self.ste = rospy.get_time()

        if stt - self.ste_2 >= 20 and lane_pixel > self.sinho_park_pix:
            self.sinho_park_check += 1
            self.ste_2 = rospy.get_time()
        if self.stoplinecheck >= 2:
            self.jungji_end_time = rospy.get_time()
        # print("jungjitime1:", self.jungji_end_time - self.jungji_time)
        if self.stoplinecheck == 2:
            self.l_jange = 0
            if self.jungji == 1:
                self.jungji_time = rospy.get_time()
                self.jungji = 0
        if (self.jungji_end_time - self.jungji_time) >= 15: #정지영
            self.rotter_blocker_flag = 0
            self.turnnel_switch = 1
            self.r_jange = 0

        # if self.stoplinecheck == 4:
            # self.drive_mode = 1

            # print("self.stoplinecheck:",self.stoplinecheck)

            # self.ste = rospy.get_time()
        print("self.stoplinecheck:", self.stoplinecheck)
        # print("self.sinho_park_check:", self.sinho_park_check)

        # print(self.stoplinecheck)

        return self.stoplinecheck, lane_pixel
        # print(lane_pixel)

        # lane_pixel =lane_pixel.reshape(-1, 2)

        # return lane_pixel

    # def stop_line_slow_mode1(self):
    #     stop_good_center_idx = self.stopline_ROI()
    #     if len(stop_good_center_idx) > self.stop_min_pix:
    #         self.drive(0)
    #         sleep(5.3)
    #         self.t4 = rospy.get_time()
    #         self.slow_mode = 2

    # def stop_line_slow_mode2(self):
    #     stop_good_center_idx = self.stopline_ROI()
    #     if len(stop_good_center_idx) > self.stop_min_pix:
    #         if self.init_cnt == False:
    #             self.stop_line_cnt += 1
    #             self.init_cnt = True
    #             self.slow_mode = 5

    # def detect_stop_line_slow_mode(self):
    #    stop_good_center_idx= self.stopline_ROI()
    #    if len(stop_good_center_idx) > self.stop_min_pix:
    #        #if self.init_cnt == False:
    #        #    self.init_cnt = True
    #            self.slow_down_flag = 1
    #    else:
    #        pass

    def follow_lane(self):  # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수\
        # self.detect_stop_line_slow_mode()
        # os.system('clear')
        # rospy.loginfo(self.sign_data)
        if self.rabastart == 1:
            self.rabastart_time = rospy.get_time()
            self.rabastart = 0
        self.nowtime = rospy.get_time()
        # print("tiemtie",self.rabastart_time)
        if self.dohyun == 1:
            if self.handle_stopline():
                return
        # rospy.loginfo(self.SOD_list)
        # rospy.loginfo(self.warninglightobj)
        # slow_down_sign
        # rospy.loginfo(self.slow_down_flag)
        elif self.nowtime - self.line_right_time >= 5:
            self.drive_mode = 1

        if self.slow_down_flag == 1:

            # rospy.loginfo("sign data :{}".format(self.sign_data))
            if self.slow_mode == 1:
                self.drive(0.33)
                self.stop_line_slow_mode1()
                rospy.loginfo("slowmode")
            elif self.slow_mode == 2:
                self.drive(0.33)
                t1 = rospy.get_time()
                rospy.loginfo("slowmode2")
                # rospy.loginfo(self.m)
                # rospy.loginfo(self.stop_line_cnt)

                # rospy.loginfo("t1 :{}, t2 : {}".format(self.slow_t1, t2))
                if t1 - self.t4 > 5:
                    self.stop_line_slow_mode2()
                if self.sign_data == 2:
                    self.m = self.sign_data
                    # self.slow_mode = 5

            elif self.stop_line_cnt >= 1:
                self.stop_line_cnt = 0
                self.init_cnt = False
                self.m = 0
                self.slow_mode = 1
                self.slow_down_flag = 0

                # self.y_list = []]
        ### red road

        elif self.red_area_ratio >= 0.01:  # 빨간색 비율 따라서 조정.
            self.red_slow()

        elif self.turnnel_mission == 1 and self.turnnel_switch == 1:
            self.turnnel_drive()
            self.turnnel_endtime = rospy.get_time()

        # 로터리 (장애물)
        elif self.rotary_left_detected == 1 and self.l_jange == 1: #self.l_jange 이거 스위치임 그냥
            # 로터리 왼쪽에서 장애물이 감지되면 정지
            self.stop()
            rospy.loginfo("Left rotary obstacle - WAITING")
            rospy.sleep(self.blocker_wait_time)  # 차단기와 동일한 대기시간 적용
            # 왼쪽 장애물 감지 초기화
            self.rotary_left_detected = 0
            rospy.loginfo("Waiting End - Starting again")
            
        elif self.rotary_right_detected == 1 and self.r_jange == 1:#self.ㄱ_jange 이거 스위치임 그냥
            # 로터리 오른쪽에서 장애물이 감지되면 정지
            self.stop()
            rospy.loginfo("Right rotary obstacle d- WAITING")
            rospy.sleep(self.blocker_wait_time)  # 차단기와 동일한 대기시간 적용
            # 오른쪽 장애물 감지 초기화
            self.rotary_right_detected = 0
            rospy.loginfo("Waiting End - Starting again")
            
            
        # 차단기
        elif (
            self.blocker_detected == 111111 # 앞에 물체 감지 
            and self.rotter_blocker_flag == 1 # 기본값 1
            and self.rotblock_start == 1 #로터리 끝나면 1
        ):  # 차단기 감지.

            if (
                self.blocker_up == 1
            ):  # 트루일 때. 차단기 보일 때. (차단기가 내려가 있을 때)
                # 차단기 앞에서 정지
                self.stop()
                rospy.loginfo("Waiting for blocker")

            elif self.blocker_up == 0:
                # 차단기가 올라가면 5초 대기 후 출발
                self.stop()
                rospy.sleep(self.blocker_wait_time)  # 설정값 5.0
                self.blocker_detected = 0  # 미션 완료 처리
                # self.drive(self.speed_lane)

        # rabacon
        elif self.rabacon_mission == 1 and self.rababa_mission == 1 and self.nowtime-self.rabastart_time >= 1.5:
            self.rabacon_drive()
            # self.current_lane = "RIGHT"
            self.mode_endtime = rospy.get_time()
            self.now_line = "LEFT"

        # obstacle
        elif self.is_safe == 0:  # WARNING 일 때
            self.y_list_sort = sorted(self.y_list, key=lambda x: x)
            # # rospy.loginfo("y: {}, {}".format(self.y_list[5], self.y_list[-5]))
            # rospy.logwarn("average:{},{}".format(statistics.mean(self.y_list[5:15]), statistics.mean(self.y_list[-15:-5])))

            # if len(self.y_list) <= 19 :
            #     rospy.loginfo(12312312312312321213123321132312312)
            #     self.stop()
            # rospy.loginfo("obstacle_stop, dynamic_flag = {}", format(self.dynamic_flag))

            if self.dynamic_obj == 1:

                self.dynamic_flag = 1
                self.static_flag = 0

            # elif abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])) >= 0.30 or self.y_list_sort[10] < -0.15:
            #    self.dynamic_flag = 1
            #    self.static_flag = 0
            # self.y_list.clear
            # rospy.loginfo("dynamic")
            # rospy.loginfo(self.y_list_sort)
            # rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))

            else:
                self.static_cnt += 1
                if self.static_cnt >= 15:
                    self.static_flag = 1
                    self.dynamic_flag = 0
                    self.static_cnt = 0
                # rospy.loginfo("static")
                # rospy.loginfo(self.y_list_sort)
                # rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))

            # dynamic
            if self.dynamic_flag == 12 and self.is_safe == False:
                rospy.logwarn("DYNAMIC OBSTACLE")
                rospy.loginfo("DYNAMIC OBSTACLE")
                self.stop()
                sleep(3)
                # self.t55 = rospy.get_time()
                self.now_line = "LEFT"

            # static obstacle
            elif (
                self.static_flag == 12
                and self.nowtime - self.t55 >= 55555555555555555555555555
            ):  # 555555555555555555555:
                rospy.loginfo("STATIC OBSTACLE")
                # if the car is driving depending on "right" window

                if self.now_line == "LEFT":
                    if self.warninglightobj == 21:
                        self.drive(0.22)
                    # rospy.logwarn("IN LEFT")
                    else:
                        if self.turn_right_flag == 0:
                            self.turn_right_t1 = rospy.get_time()
                            self.turn_right_flag = 1
                        t2 = rospy.get_time()
                        # rospy.loginfo("turn time{}, {}".format(self.turn_right_t1,t2))

                        while t2 - self.turn_right_t1 <= 1.4:  # 0.8
                            self.change_line_right()
                            t2 = rospy.get_time()
                        while t2 - self.turn_right_t1 <= 1.65:
                            self.change_line_left()
                            t2 = rospy.get_time()
                        # self.current_lane = "RIGHT"
                        self.leftcurrent_cnt = 0
                        self.rightcurrent_cnt = 0
                        self.now_line = "RIGHT"
                        self.static_flag = 0
                        self.turn_right_flag = 0
                    # self.follow_lane()

                elif self.now_line == "RIGHT":
                    if self.warningleftobj == 1:
                        self.drive(0.22)
                    else:
                        # rospy.logwarn("IN RIGHT")
                        if self.turn_left_flag == 0:
                            self.turn_left_t1 = rospy.get_time()
                            self.turn_left_flag = 1
                        t2 = rospy.get_time()
                        # rospy.loginfo("turn time{}, {}".format(self.turn_left_t1,t2))

                        while t2 - self.turn_left_t1 <= 1.69:
                            self.change_line_left()
                            t2 = rospy.get_time()
                        while t2 - self.turn_left_t1 <= 2.2:
                            self.change_line_right()
                            t2 = rospy.get_time()
                        # self.current_lane = "LEFT"
                        self.leftcurrent_cnt = 0
                        self.rightcurrent_cnt = 0
                        self.now_line = "LEFT"
                        self.static_flag = 0
                        self.turn_left_flag = 0
            else:
                pass
                # self.drive(4)  # 여름방학 정적 급발진 이유
        # elif self.change_flag == 1 :
        #    if self.initright_time ==  False:
        #        self.turn_left_t10 = rospy.get_time()
        #        self.initright_time = True
        #    t22 = rospy.get_time()
        #    while t22-self.turn_left_t10 <= 1:
        #        self.change_line_left()
        #        t22 = rospy.get_time()
        #        self.initright_time = False
        #
        #    if self.change_slow_drivemode ==False:
        #        self.turn_left_t11 = rospy.get_time()
        #        self.change_slow_drivemode == True
        #    while t22 - self.turn_left_t11 < 5.5:
        #        self.change_slow_drive(0.3)
        #        t22 = rospy.get_time()
        #
        #    self.change_slow_drivemode == False
        #    self.change_flag = 0
        #    self.current_cnt = 0
        # 주차 미션
        
        # elif self.nowtime - self.line_right_time >= 5:
        #     self.drive_mode = 1
        
        elif self.parking_mode == 1 and self.sinho_park_check == 2:
            rospy.loginfo("Parking Start")
            if self.park_start_flag == 0:
                t1 = rospy.get_time()
                self.park_start_flag = 1
            t2 = rospy.get_time()
            while t2 - t1 <= 2:  # 2     #1.82
                self.parking_start()
                t2 = rospy.get_time()
            t3 = rospy.get_time()
            t4 = rospy.get_time()

            while t4 - t3 <= 2.1:  # 1.85
                self.parking_back_1()
                t4 = rospy.get_time()

            t5 = rospy.get_time()
            t6 = rospy.get_time()
            while t6 - t5 <= 1.95:
                self.parking_back_2()
                t6 = rospy.get_time()

            t77 = rospy.get_time()
            t76 = rospy.get_time()
            while t77 - t76 <= 10:
                self.park_korean_stop()
                t77 = rospy.get_time()

            t7 = rospy.get_time()
            t8 = rospy.get_time()
            while t8 - t7 <= 1.3:
                self.parking_out_1()
                t8 = rospy.get_time()

            t9 = rospy.get_time()
            t10 = rospy.get_time()
            while t10 - t9 <= 1.3:
                self.parking_out_2()
                t10 = rospy.get_time()
            self.parking_mode = 0

        # no obstalce, no rabacon, no slow sign, no change line just drive
        elif self.nowtime - self.turnnel_endtime < 0.3:
            rospy.loginfo("turnnelend")
            self.turnnel_switch = 0

        elif self.nowtime - self.mode_endtime < 0.3:
            # self.change_line_right()
            self.change_line_left()
            rospy.loginfo("rabaend")
            self.rababa_mission = 0
            self.rotblock_start = 1
            self.l_jange = 1
            self.r_jange = 1
            self.nowtime = rospy.get_time()
        else:
            # rospy.loginfo("DEFAULT DRIVE!!!!!!!!!")

            self.error_lane = 160 - self.acenter_fit_x
            # print("acerr", self.acenter_fit_x)
            pid_angle = self.pid_control(self.error_lane)
            # rospy.loginfo(pid_angle * 0.003)
            self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
            self.moter_msg.drive.steering_angle = pid_angle * 0.02  # 0.03)

            # self.error_lane * 0.003 # 대상꺼는 0.003 error 정도에 따라서 조향
            # print("steer", self.moter_msg.drive.steering_angle)
            self.moter_msg.drive.speed = (
                self.speed_lane
            )  # - abs( pid_angle * 0.000054) #main_class_run이 데이터를 보낼 때의 시점
            self.drive_pub.publish(self.moter_msg)  #  하는 부분

    # if the car find any obstacle, the car should stop.
    def stop(self):
        # if rabacon_mission == 0: # 내가 따로 넣은 코드
        # self.moter_msg = AckermannDriveStamped()

        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.steering_angle = -0.3
        self.moter_msg.drive.speed = self.speed_stop
        self.drive_pub.publish(self.moter_msg)  # 실제 publish 하는 부분

    # rospy.loginfo("Stop Vehicle!")
    # self.stop_cnt += 1
    # print("STOP_CNT{}".format(self.stop_cnt))

    ####################################################################################
    # protect child
    def child_sign_callback(self, _data):
        try:
            # print("sign:::", _data)
            if _data.data == 0:
                self.child_cnt += 1
                if self.lys == 1:
                    self.line_right_time = rospy.get_time()
                    self.lys = 0
                if self.child_cnt >= 8:
                    self.drive_mode = 2
                    # print("selfdrivemode", self.drive_mode)
                    # self
                
                    # self.sign_data = 3
                    # self.sign_data = _data.data
                    # self.slow_down_flag = 1
                    # self.child_cnt = 0
                    # rospy.loginfo(self.sign_data)

            elif _data.data == 4:
                self.child_cnt += 1
                if self.child_cnt >= 8:
                    self.drive_mode = 1
                    # print("selfdrivemode", self.drive_mode)
                    # self.sign_data = _data.data
                    # self.slow_down_flag = 1
                    # self.child_cnt = 0
                    # rospy.loginfo(self.sign_data)
            else:
                pass
                # self.sign_data = 0

        except:
            pass

    # def change_linecnt(self):
    #    if self.current_lane == "RIGHT":
    #        self.rightcurrent_cnt += 1
    #        if  self.rightcurrent_cnt >= 50 :
    #            self.change_flag = 1
    #            self.current_cnt = 0
    #            self.now_line = "RIGHT"
    #            self.leftcurrent_cnt =0
    #            self.rightcurrent_cnt= 0
    #    elif self.current_lane == "LEFT":
    #        self.leftcurrent_cnt += 1
    #        if  self.leftcurrent_cnt >= 50 :
    #            self.change_flag = 1
    #            self.current_cnt = 0
    #            self.now_line = "LEFT"
    #            self.leftcurrent_cnt =0
    #            self.rightcurrent_cnt= 0

    def change_slow_drive(self, speed):
        self.error_lane = 360 - self.acenter_fit_x
        pid_angle = self.pid_control(self.error_lane)
        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.steering_angle = (
            pid_angle * 0.0057
        )  # self.error_lane * 0.003 # 대상꺼는 0.003 error 정도에 따라서 조향
        self.moter_msg.drive.speed = speed  # main_class_run이 데이터를 보낼 때의 시점
        self.drive_pub.publish(self.moter_msg)

    def change_line_left(self):
        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.speed = self.speed_turn
        self.moter_msg.drive.steering_angle = 0.2
        self.drive_pub.publish(self.moter_msg)

    def change_line_right(self):

        # rospy.loginfo("change_right!!!!!!!!!!!!!!!!!")
        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.speed = self.speed_turn
        self.moter_msg.drive.steering_angle = -0.27  # (라바콘 탈출 시 차선이탈 방지용)
        self.drive_pub.publish(self.moter_msg)

    # turnnel
    def turnnel_drive(self):

        # rospy.loginfo("turnnel_drive!!!!!!!!!!!!!!!!!")
        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.speed = self.speed_turnnel
        self.moter_msg.drive.steering_angle = self.turnnel_mission_flag
        rospy.loginfo(self.moter_msg.drive.steering_angle)
        self.drive_pub.publish(self.moter_msg)

    ####################################################################################
    # Rabacon Mission
    def rabacon_drive(self):

        rospy.loginfo("rabacon_drive!!!!!!!!!!!!!!!!!")
        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.speed = self.speed_rabacon
        self.moter_msg.drive.steering_angle = self.rabacon_mission_flag * -1.0 * 1.8#1.8
        # print("rabasteer: ",self.rabacon_mission_flag * -1.0 * 1.8 )
        # rospy.lograbacon_driveinfo(self.moter_msg.drive.steering_angle)
        self.drive_pub.publish(self.moter_msg)

    ####################################################################################
    # red_slow Mission
    def red_slow(self):
        self.error_lane = 160 - self.acenter_fit_x
        pid_angle = self.pid_control(self.error_lane)
        self.moter_msg.header.stamp = rospy.Time.now()  # 이 데이터를 보낼 때의 시점
        self.moter_msg.drive.speed = self.speed_red_slow
        self.moter_msg.drive.steering_angle = pid_angle * 0.03
        self.drive_pub.publish(self.moter_msg)


def run():

    rospy.init_node("main_class_run")
    control = Controller()
    rospy.Timer(rospy.Duration(1.0 / 60.0), control.timer_callback)
    rospy.spin()


if __name__ == "__main__":
    run()

    #########right lane, left lane#########
    # # 현재 시간 저장
    # stop_start_time = rospy.get_time()

    # # 처음 6초는 정지만 유지
    # while rospy.get_time() - stop_start_time < 5.5:
    #     self.drive(0)
    #     sleep(0.1)

    # drive_mode 변경 및 주행
    # self.drive_mode = 1  # 우측 차선 모드로 변경

    # # 우측 차선 주행을 위한 시간 제공
    # drive_start_time = rospy.get_time()
    # while rospy.get_time() - drive_start_time < 7.59:
    #     self.error_lane = 160 - self.acenter_fit_x  # 차선 위치 오차 계산
    #     pid_angle = self.pid_control(self.error_lane)
    #     self.moter_msg.header.stamp = rospy.Time.now()
    #     self.moter_msg.drive.steering_angle = pid_angle * 0.02
    #     self.moter_msg.drive.speed = 0.3
    #     self.drive_pub.publish(self.moter_msg)
    #     sleep(0.05)

    # self.drive_mode = 2  # 다시 좌측 차선 모드로 복귀
    # rospy.loginfo("Returned to left lane mode")