#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointCloud
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, fabs, tan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import tf
import sys
import roslib
import numpy as np
import copy 
from sensor_msgs.msg import JointState
from std_msgs.msg import Time

exp_name = sys.argv[1]

class data_save():

    # constructor class
    def __init__(self):

        #init node
        rospy.init_node('data_register', anonymous=True)

        #subscribers
        self.sub_gt = rospy.Subscriber('/mocap_node/espeleo/Odom', Odometry, self.callback_gt)
        self.sub_lidar = rospy.Subscriber('/ekf_loam/laser_odom_to_init', Odometry, self.callback_lidar)
        self.sub_integ = rospy.Subscriber('/ekf_loam/integrated_to_init', Odometry, self.callback_integrated)
        self.sub_integ = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.sub_integ = rospy.Subscriber('/os1_cloud_node/points', PointCloud2, self.callback_ouster)

        #path name
        # self.path_name = '/home/gilmarcoro/catkin_ws/experiments/features_test/'
        self.path_name = '/home/gilmarcoro/catkin_ws/experiments/fuzzy_final_project/'
        self.time_init = rospy.Time.now()
        self.ouster = False
        self.msg_gt = ''
        
        #rate
        f_hz = float(200)
        rate = rospy.Rate(f_hz)      

        # for time
        k = 1
        
        #loop principal
        print "Data Register is runnig: \n"
        
        #loop
        while not rospy.is_shutdown():    
            # output screen
            s = 'tempo = ' + str(k*(1/f_hz))        # string for output
            sys.stdout.write(s)                     # just print
            sys.stdout.flush()                      # needed for flush when using \x08
            self.backspace(len(s))  

            k = k + 1
            #gastando tempo do rate
            rate.sleep()

        print "Data register end!"

    def backspace(self, n):
        sys.stdout.write((b'\x08' * n).decode()) 


    #callback functions
    def callback_gt(self, data):
        odom = data

        self.msg_gt = ''
        self.msg_gt += str(odom.header.seq) + ','
        self.msg_gt += str(odom.header.stamp.to_sec()) + ','
        self.msg_gt += str(odom.pose.pose.position.x) + ','
        self.msg_gt += str(odom.pose.pose.position.y) + ','
        self.msg_gt += str(odom.pose.pose.position.z) + ','
        self.msg_gt += str(odom.pose.pose.orientation.x) + ','
        self.msg_gt += str(odom.pose.pose.orientation.y) + ','
        self.msg_gt += str(odom.pose.pose.orientation.z) + ','
        self.msg_gt += str(odom.pose.pose.orientation.w) + '\n'
           
        
        return

    def callback_lidar(self, data):
        odom = data

        msg_s = ''
        msg_s += str(odom.header.seq) + ','
        msg_s += str(odom.header.stamp.to_sec()) + ','
        msg_s += str(odom.pose.pose.position.x) + ','
        msg_s += str(odom.pose.pose.position.y) + ','
        msg_s += str(odom.pose.pose.position.z) + ','
        msg_s += str(odom.pose.pose.orientation.x) + ','
        msg_s += str(odom.pose.pose.orientation.y) + ','
        msg_s += str(odom.pose.pose.orientation.z) + ','
        msg_s += str(odom.pose.pose.orientation.w) + ','
        msg_s += str(odom.twist.twist.linear.x) + ','
        msg_s += str(odom.twist.twist.angular.x) + ','
        msg_s += str(odom.twist.twist.linear.y) + ','
        msg_s += str(odom.twist.twist.angular.y) + '\n'

        #salando no arquivo
        if (self.ouster):
            file_object = open(self.path_name+'lidar_odometry_' + str(exp_name) + '.txt', 'a')        
            file_object.write(msg_s)
            file_object.close()

            file_objectgt = open(self.path_name+'ground_truth_' + str(exp_name) + '.txt', 'a')        
            file_objectgt.write(self.msg_gt)
            file_objectgt.close()
        
        return

    def callback_integrated(self, data):
        odom = data

        msg_s = ''
        msg_s += str(odom.header.seq) + ','
        msg_s += str(odom.header.stamp.to_sec()) + ','
        msg_s += str(odom.pose.pose.position.x) + ','
        msg_s += str(odom.pose.pose.position.y) + ','
        msg_s += str(odom.pose.pose.position.z) + ','
        msg_s += str(odom.pose.pose.orientation.x) + ','
        msg_s += str(odom.pose.pose.orientation.y) + ','
        msg_s += str(odom.pose.pose.orientation.z) + ','
        msg_s += str(odom.pose.pose.orientation.w) + '\n'

        #salando no arquivo
        if (self.ouster):
            file_object = open(self.path_name+'integrated_odometry_' + str(exp_name) + '.txt', 'a')        
            file_object.write(msg_s)
            file_object.close()
        
        return

    def callback_odom(self, data):
        odom = data

        msg_s = ''
        msg_s += str(odom.header.seq) + ','
        msg_s += str(odom.header.stamp.to_sec()) + ','
        msg_s += str(odom.pose.pose.position.x) + ','
        msg_s += str(odom.pose.pose.position.y) + ','
        msg_s += str(odom.pose.pose.position.z) + ','
        msg_s += str(odom.pose.pose.orientation.x) + ','
        msg_s += str(odom.pose.pose.orientation.y) + ','
        msg_s += str(odom.pose.pose.orientation.z) + ','
        msg_s += str(odom.pose.pose.orientation.w) + '\n'

        #salando no arquivo
        if (self.ouster):
            file_object = open(self.path_name+'wheel_odom_' + str(exp_name) + '.txt', 'a')        
            file_object.write(msg_s)
            file_object.close()
        
        return

    def callback_ouster(self, data):
        self.ouster = True
        
        return

if __name__ == '__main__':
    try:
        data_save()   
        
    except rospy.ROSInterruptException:
        pass
    
        

    
