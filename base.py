#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
import math
import serial
import time
import numpy as np


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(1)

class Robot:
    def __init__(self):
        rospy.init_node('robot', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.stop_timer = 0
        self.min_distance = [0] * 8
        self.THRESHOLD = 1.0
        self.lidar_enabled = True  # 라이다 데이터 처리를 활성화합니다.
        self.turn_sent = False  # 새로운 속성 추가
        self.count = False

    def lidar_callback(self, scan_data):
    
        lb = scan_data.ranges[-55] 
        rb = scan_data.ranges[45]   
        left = scan_data.ranges[-130]
        right = scan_data.ranges[130]  
        lf = scan_data.ranges[300]
        rf = scan_data.ranges[200]  
        front = scan_data.ranges[250] 
        back = scan_data.ranges[-5]  
        distances = [front, lf, left, lb, back, rb, right, rf]
        
        self.min_distance = distances
        if not self.lidar_enabled :  # lidar_enabled가 False이면 처리를 건너뜁니다.
            return
        
         
    def cancel(self):
        ser.write(b'O')
        print("pause")
        time.sleep(1)

    def parallel_left(self):
        
        ser.write(b'Z')
        print("paraller_left")
        time.sleep(0.2)
        

    def parallel_right(self):
        
        ser.write(b'X')
        print("paraller_right")
        time.sleep(0.2)
       
    def short_forward(self):
        
        ser.write(b'C')
        print("short_forward")
        time.sleep(1)
       

    def forward(self):
        
        ser.write(b'F')
        print("forward")
        time.sleep(1)
        

    def left(self):
        
        ser.write(b'L')
        print("left")
        time.sleep(1.5)
        

    def right(self):
        
        ser.write(b'R')
        print("right")
        time.sleep(1.5)
        

    def stop(self):
        
        ser.write(b'S')
        print("stop")
        time.sleep(1)


    def parallel(self, side, THRESHOLD):
            side_dist1, side_dist2 = 0, 0  # Add this line
        
            distances = self.min_distance
            front, lf, left, lb, back, rb, right, rf = distances
            
            if side == 'rfrb':
                side_dist1, side_dist2 = rf, rb
                parallel_func1, parallel_func2 = self.parallel_right, self.parallel_left
            elif side == 'lflb':
                side_dist1, side_dist2 = lf, lb
                parallel_func1, parallel_func2 = self.parallel_left, self.parallel_right
            elif side == 'backward':  # 추가된 부분
                side_dist1, side_dist2 = lb, rb
                parallel_func1, parallel_func2 = self.parallel_left, self.parallel_right

            #print("{} - side_dist1: {}, side_dist2: {}, abs(side_dist1-side_dist2): {}".format(side, side_dist1, side_dist2, abs(side_dist1-side_dist2)))

            if abs(side_dist1 - side_dist2) < 0.3:
                self.short_forward()
                
                print("short_forward")
            
            elif abs(side_dist1 > side_dist2):
                parallel_func1()
                print("parrel_func1")

            elif abs(side_dist1 < side_dist2):
                parallel_func2()
                print("parrel_func2")


    # def turn_left(self):
        
    #     self.left()# 90도 회전
    #     time.sleep(3)
    #     self.parallel('rfrb', self.THRESHOLD)
        
    #     self.left()
    #     time.sleep(3)
    #     self.stop()

    # def turn_right(self):
    #     if not self.turn_sent:
    #             if self.prev_command != "right":  # 이전 명령어와 다를 경우에만 전송합니다.
    #                 self.right()# 90도 회전
    #                 self.prev_command = "right"
    #             self.turn_sent = True
    #     self.parallel('lflb', self.THRESHOLD)
    #     self.right()
    #     time.sleep(3)
    #     self.stop()


    def zigzag(self,min_distance):
        front, lf, left, lb, back, rb, right, rf = self.min_distance
#self.count가 false 이면 좌회전        
        if self.count == False:
            if front < self.THRESHOLD and front > 0.0:
                print("Front: {:.2f}".format(front))
                self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.
                self.stop()
                #self.cancel()
                time.sleep(1)
                if not self.turn_sent:  # 회전 명령이 전송되지 않은 경우에만 전송합니다.
                    self.left()
                    self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.

                self.stop()

    #라이다 다시키기
                while True:
                    front, lf, left, lb, back, rb, right, rf = self.min_distance
                    self.lidar_enabled = True  # 라이다 데이터 처리를 다시 활성화합니다.
                    print ("rf: {:.2f}, rb: {:.2f}".format(rf, rb)) 
    #라이다 다시끄기
                    #self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.            
    #수식 계산  
                    if abs(rf-rb)<0.1 :
                       # if not self.turn_sent:
                        self.short_forward()
                        #    self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.
                        time.sleep(1)  # 좌회전 동안 대기합니다.
                        #self.stop()
                        print("escape")
                        self.cancel()
                        #self.lidar_enabled = True
                        break

                    elif abs(rf > rb):
                        print("enterace")
                        #if not self.turn_sent:
                        self.parallel_right()
                        #    self.turn_sent = True  

                        self.cancel()

                    elif abs(rb > rf):
                        print("outtranc")
                        #if not self.turn_sent:  
                        self.parallel_left()
                        #self.turn_sent = True
                        #
                        self.cancel()

                print("last left")
                self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.
                self.stop()
                time.sleep(1)
                #if not self.turn_sent:  # 회전 명령이 전송되지 않은 경우에만 전송합니다.
                self.left()
                #self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.                
                self.stop()
                self.lidar_enabled = True

    #back check            
                while True:
                    front, lf, left, lb, back, rb, right, rf = self.min_distance
                    self.lidar_enabled = True  # 라이다 데이터 처리를 다시 활성화합니다.
                    print ("lb: {:.2f}, rb: {:.2f}".format(lb, rb)) 
    #라이다 다시끄기
                    #self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.            
    #수식 계산  
                    if abs(lb-rb)<0.1 :
                        #if not self.turn_sent:
                        self.short_forward()
                         #   self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.
                        time.sleep(1)  # 좌회전 동안 대기합니다.
                        #self.stop()
                        print("back escape")
                        self.cancel()
                        #self.lidar_enabled = True
                        break

                    elif abs(lb > rb):
                        print("enterace")
                        #if not self.turn_sent:
                        self.parallel_left()
                        #    self.turn_sent = True  

                        self.cancel()

                    elif abs(rb > lb):
                        print("outtranc")
                        #if not self.turn_sent:  
                        self.parallel_right()
                        #self.turn_sent = True
                        #
                        self.cancel()
                print("back check")
                self.lidar_enabled = True
                self.count =True

            else:
                self.turn_sent = False
                self.forward()
    ####################################################zigzag_left_complete#################

        elif self.count == True:
#좌회전
            if front < self.THRESHOLD and front > 0.0:
                print("Front: {:.2f}".format(front))
                self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.
                self.stop()
                #self.cancel()
                time.sleep(1)
                #if not self.turn_sent:  # 회전 명령이 전송되지 않은 경우에만 전송합니다.
                self.right()
                #    self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.
                self.stop()

    #라이다 다시키기
                while True:
                    front, lf, left, lb, back, rb, right, rf = self.min_distance
                    self.lidar_enabled = True  # 라이다 데이터 처리를 다시 활성화합니다.
                    print ("lf: {:.2f}, lb: {:.2f}".format(lf, lb)) 
    #라이다 다시끄기
                    #self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.            
    #수식 계산  
                    if abs(lf-lb)<0.1:
                        
                        self.short_forward()
                        # self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.
                        time.sleep(1)  # 좌회전 동안 대기합니다.
                        self.stop()
                        print("escape")
                        self.cancel()
                        #self.lidar_enabled = True
                        break

                    elif abs(lf > lb):
                        print("enterace")
                        #if not self.turn_sent:
                        self.parallel_left()
                        #    self.turn_sent = True  

                        self.cancel()

                    elif abs(lb > lf):
                        print("outtranc")
                        #if not self.turn_sent:  
                        self.parallel_right()
                        #self.turn_sent = True
                        #
                        self.cancel()

                print("last right")
                self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.
                self.stop()
                time.sleep(1)
                #if not self.turn_sent:  # 회전 명령이 전송되지 않은 경우에만 전송합니다.
                self.right()
                #self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.                
                self.stop()
                self.lidar_enabled = True

    #back check            
                while True:
                    front, lf, left, lb, back, rb, right, rf = self.min_distance
                    self.lidar_enabled = True  # 라이다 데이터 처리를 다시 활성화합니다.
                    print ("lb: {:.2f}, rb: {:.2f}".format(lb, rb)) 
    #라이다 다시끄기
                    #self.lidar_enabled = False  # 라이다 데이터 처리를 잠시 중단합니다.            
    #수식 계산  
                    if abs(lb-rb)<0.1:
                        #if not self.turn_sent:
                        print("back escape")
                        self.short_forward()
                        #    self.turn_sent = True  # 회전 명령이 전송되었음을 나타냅니다.
                        time.sleep(1)  # 좌회전 동안 대기합니다.
                        self.stop()
                        
                        self.cancel()
                        #self.lidar_enabled = True
                        break

                    elif abs(lb > rb):
                        print("enterace")
                        #if not self.turn_sent:
                        self.parallel_left()
                        #    self.turn_sent = True  
                        self.cancel()

                    elif abs(rb > lb):
                        print("outtranc")
                        #if not self.turn_sent:  
                        self.parallel_right()
                        #self.turn_sent = True
                        self.cancel()
                print("back check")
                self.lidar_enabled = True
                self.count =False

            else:
                self.turn_sent = False
                self.forward()
    ####################################################zigzag_right_complete#################





    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.zigzag(self.min_distance)
            rate.sleep()
        

if __name__ == "__main__":
    try:
        rospy.loginfo("Robot node started.")
        node = Robot()
        node.run()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        ser.close()  # 시리얼 통신 닫기