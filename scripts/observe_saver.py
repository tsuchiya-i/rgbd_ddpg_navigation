#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import cv2
import sys, os
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time

global odom_x,odom_y,odom_a
global goal_dist, goal_angle

goal_x = 10
goal_y = 0

def callback(data):
    global observe_data
    global goal_dist, goal_angle

    np_arr = np.fromstring(data.data, np.uint8)
    np_array = np.array(np_arr)
    np_array = np_array.reshape(data.height, data.width*2)

    middle_line = np_array[data.height//2]
    d_width = int(data.width/4.5)
    center = data.width
    observe_lines = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    print("=================================")

    observe_lines[0] = (middle_line[center])+255*(middle_line[center-1])
    observe_lines[0] = observe_lines[0]/1000

    for i in range(4):
        observe_lines[i+1] = middle_line[center-d_width*(i+1)]+255*middle_line[center-d_width*(i+1)-1]
        observe_lines[i+1] = observe_lines[i+1]/1000

    for i in range(4):
        ri = 4-i
        observe_lines[i+5] = middle_line[center+d_width*ri]+255*middle_line[center+d_width*ri-1]
        observe_lines[i+1] = observe_lines[i+1]/1000

    observe_lines = np.array(observe_lines)

    goal_point = np.array([goal_x, goal_y])
    odom_point = np.array([odom_x, odom_y])
    goal_dist = np.linalg.norm(goal_point-odom_point)

    observe_data = np.append(observe_lines,goal_dist)
    
    t_theta = np.angle(complex(goal_x-odom_x, goal_y-odom_y))
    if t_theta < 0:
        t_theta = math.pi + (math.pi+t_theta)
    theta = t_theta
    if odom_a < math.pi:
        if t_theta > odom_a+math.pi:
            theta=t_theta-2*math.pi
    if odom_a > math.pi:
        if 0 < t_theta < odom_a-math.pi:
            theta=t_theta+2*math.pi
    goal_angle = theta-odom_a

    observe_data = np.append(observe_data,goal_angle)

    print(observe_data)

    #np.savetxt('~/my_workspace/src/beginner_tutorials/observe_data.csv',observe_data)
    np.savetxt(os.path.dirname(__file__) + "/gym_pathplan/envs/observe_data.csv",observe_data)
    
    cv2.imshow("test", np_array)
    cv2.waitKey(1)

def callback_odom(data):
    global odom_x,odom_y,odom_a

    if not data.pose.pose.position.x == data.pose.pose.position.y:
        odom_x = data.pose.pose.position.x
        odom_y = data.pose.pose.position.y
        odom_a = data.pose.pose.orientation.z * (math.pi/2)



def listener():
    rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber("camera/depth/image_rect_raw/compressedDepth", CompressedImage, callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)
    rospy.Subscriber("/rtabmap/odom",Odometry , callback_odom)
    #rospy.Subscriber("camera/color/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
