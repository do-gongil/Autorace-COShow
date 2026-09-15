#!/usr/bin/env python
#-*- coding: utf-8 -*-
# ROS_NAMESPPACE=usb_cam rosrun image_proc image_proc
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

class cameraReceiber():
    def __init__(self):
        self.window_height = np.int32(480 / 20)
        #self.window_height = np.int32(img.shape[0] / self.nwindows)
        self.initialized  =False
        self.nothing_flag = False
        self.margin = 70
        self.nwindows = 20


    #def camera_callback(self, img):
        #lane_image = cv2.inRange(crop_img, lower_lane,upper_lane)
        
        #sliding_window_msg = self.bridge.cv2_to_compressed_imgmsg(sliding_window_img)
        #cv2.imshow("Daelimcar_camera", sliding_window_img )
        #cv2.imshow("ori_img", img )
        #cv2.imshow("th2", binary_line )
#
        ##cv2.imshow("Daelimcar_camera1", channel_1)
        ##cv2.imshow("Daelimcar_camera2", channel_2)
        #cv2.imshow("Daelimcar_camera3", channel_3)
        ##cv2.imshow("orimg",cv_image)   
        #cv2.waitKey(1)


    def img_warp(self, img):
        h, w = (img.shape[0], img.shape[1])
        self.img_y, self.img_x = h, w
        
        source = np.float32([[240, 290], [400, 290], [70, 395], [570, 395]])
        destination = np.float32([[0, 0], [640, 0], [101, h], [600, h]])

        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        #minv = cv2.getPerspectiveTransform(destination, source)
        _image = cv2.warpPerspective(img, transform_matrix, (w, h))

        return _image



    def detect_nothing(self):
        self.nothing_left_x_base = round(self.img_x * 0.140625)
        self.nothing_right_x_base = self.img_x - round(self.img_x * 0.140625)

        self.nothing_pixel_left_x = np.zeros(self.nwindows) + round(
            self.img_x * 0.140625
        )
        self.nothing_pixel_right_x = (
            np.zeros(self.nwindows) + self.img_x - round(self.img_x * 0.140625)
        )
        self.nothing_pixel_y = [
            round(self.window_height / 2) * index for index in range(0, self.nwindows)
        ]

    def window_search(self, binary_line):

        bottom_half_y = binary_line.shape[0] / 2
        histogram = np.sum(binary_line[int(bottom_half_y) :, :], axis=0)
        # 히스토그램을 절반으로 나누어 좌우 히스토그램의 최대값의 인덱스를 반환합니다.
        midpoint = np.int32(histogram.shape[0] / 2)
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint
    
        if left_x_base == 0:
            left_x_current = self.nothing_left_x_base
        else:
            left_x_current = left_x_base
        if right_x_base == midpoint: 
            right_x_current = self.nothing_right_x_base
        else:
            right_x_current = right_x_base

        #1차원을 3차원으로 나타냄왜냐하면 나중에 화면상에 윈도우랑 차선색깔을 나타내기 위해서
        out_img = np.dstack((binary_line, binary_line, binary_line)) * 255 

        ## window parameter
        # 적절한 윈도우의 개수를 지정합니다.
        nwindows = self.nwindows
        # 개수가 너무 적으면 정확하게 차선을 찾기 힘듭니다.
        # 개수가 너무 많으면 연산량이 증가하여 시간이 오래 걸립니다.
        window_height = self.window_height
        # 윈도우의 너비를 지정합니다. 윈도우가 옆 차선까지 넘어가지 않게 사이즈를 적절히 지정합니다.
        #margin = 30
        # 탐색할 최소 픽셀의 개수를 지정합니다.
        min_pix = round((self.margin * 2 * window_height) * 0.042) # h:480 기준 24픽셀 

        lane_pixel = binary_line.nonzero()
        lane_pixel_y = np.array(lane_pixel[0])
        lane_pixel_x = np.array(lane_pixel[1])

        # pixel index를 담을 list를 만들어 줍니다.
        left_lane_idx = []
        right_lane_idx = []
        #b_good_left_idx = []
        #b_good_right_idx = []
        #good_left_idx = []
        #good_right_idx = []
        # Step through the windows one by one
        for window in range(nwindows):
            
            # window boundary를 지정합니다. (가로)
            win_y_low = binary_line.shape[0] - (window + 1) * window_height
            win_y_high = binary_line.shape[0] - window * window_height
            # print("check param : \n",window,win_y_low,win_y_high)

            # position 기준 window size
            win_x_left_low = left_x_current - self.margin   
            win_x_left_high = left_x_current + self.margin
            win_x_right_low = right_x_current - self.margin
            win_x_right_high = right_x_current + self.margin

          
             #window 시각화입니다.
            if left_x_current != 0:
                cv2.rectangle(
                    out_img,
                    (win_x_left_low, win_y_low),
                    (win_x_left_high, win_y_high),
                    (0, 255, 0),
                    2,
                )
            if right_x_current != midpoint:
                cv2.rectangle(
                    out_img,
                    (win_x_right_low, win_y_low),
                    (win_x_right_high, win_y_high),
                    (0, 0, 255),
                    2,
                )
            
            # 왼쪽 오른쪽 각 차선 픽셀이 window안에 있는 경우 index를 저장합니다.
            good_left_idx = (
                (lane_pixel_y >= win_y_low)
                & (lane_pixel_y < win_y_low+10)
                & (lane_pixel_x >= win_x_left_low)
                & (lane_pixel_x < win_x_left_high)
            ).nonzero()[0]

            good_right_idx = (
                (lane_pixel_y >= win_y_low)
                & (lane_pixel_y < win_y_low+10)
                & (lane_pixel_x >= win_x_right_low)
                & (lane_pixel_x < win_x_right_high)
            ).nonzero()[0]
            #if len(b_good_right_idx) < min_pix and len(b_good_left_idx) < min_pix :
            #    pass
            #if len(b_good_right_idx) < min_pix:
            #    b_good_right_idx = b_good_left_idx
            #if len(good_left_idx) < min_pix:    
            #    b_good_left_idx = b_good_right_idx
#
#
            #good_left_idx.append(b_good_left_idx)
            #good_right_idx.append(b_good_right_idx) 
        
            # Append these indices to the lists
            left_lane_idx.append(good_left_idx)
            right_lane_idx.append(good_right_idx)

            # window내 설정한 pixel개수 이상이 탐지되면, 픽셀들의 x 좌표 평균으로 업데이트 합니다.
            if len(good_left_idx) > min_pix:
                left_x_current = np.int32(np.mean(lane_pixel_x[good_left_idx]))
            if len(good_right_idx) > min_pix:
                right_x_current = np.int32(np.mean(lane_pixel_x[good_right_idx]))

        # np.concatenate(array) => axis 0으로 차원 감소 시킵니다.(window개수로 감소)
        left_lane_idx = np.concatenate(left_lane_idx)
        right_lane_idx = np.concatenate(right_lane_idx)

        # window 별 좌우 도로 픽셀 좌표입니다.
        left_x = lane_pixel_x[left_lane_idx]
        left_y = lane_pixel_y[left_lane_idx]
        right_x = lane_pixel_x[right_lane_idx]
        right_y = lane_pixel_y[right_lane_idx]

        # 좌우 차선 별 2차함수 계수를 추정합니다.
        if len(left_x) == 0 and len(right_x) == 0:
            left_x = self.nothing_pixel_left_x
            left_y = self.nothing_pixel_y
            right_x = self.nothing_pixel_right_x
            right_y = self.nothing_pixel_y
        else:
            if len(left_x) < min_pix:
                left_x = right_x - 320  #- round(self.img_x / 2)
                left_y = right_y
            elif len(right_x) < min_pix:
                right_x = left_x - 320  # + round(self.img_x / 2)
                right_y = left_y

        left_fit = np.polyfit(left_y, left_x, 2)
        right_fit = np.polyfit(right_y, right_x, 2)
        # 좌우 차선 별 추정할 y좌표입니다.
        plot_y = np.linspace(0, binary_line.shape[0] - 1, 5)
        # 좌우 차선 별 2차 곡선을 추정합니다.
        left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
        right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]
        center_fit_x = (right_fit_x + left_fit_x) / 2

        aleft_fit_x = left_fit[0] * 400**2 + left_fit[1] * 400 + left_fit[2]
        aright_fit_x = right_fit[0] * 400**2 + right_fit[1] * 400 + right_fit[2]
        acenter_fit_x = (aright_fit_x + aleft_fit_x) / 2

        # # window안의 lane을 black 처리합니다.
        #out_img[lane_pixel_y[left_lane_idx], lane_pixel_x[left_lane_idx]] = (0, 0, 0)
        #out_img[lane_pixel_y[right_lane_idx], lane_pixel_x[right_lane_idx]] = (0, 0, 0)

        # 양쪽 차선 및 중심 선 pixel 좌표(x,y)로 변환합니다.
        center = np.asarray(tuple(zip(center_fit_x, plot_y)), np.int32)
        right = np.asarray(tuple(zip(right_fit_x, plot_y)), np.int32)
        left = np.asarray(tuple(zip(left_fit_x, plot_y)), np.int32)
        
        cv2.polylines(out_img, [left], False, (0, 0, 255), thickness=5)
        cv2.polylines(out_img, [right], False, (0, 255, 0), thickness=5)
        cv2.polylines(out_img, [center], False, (255, 0, 0), thickness=5)

        sliding_window_img = out_img
        return sliding_window_img, left, right, center, left_x, left_y, right_x, right_y,center_fit_x,acenter_fit_x
    
         
#def run():
#    rospy.init_node("slidewin")
#    new_class = cameraReceiber()
#    rospy.spin()
#
#if __name__ == '__main__':
#    run()
 