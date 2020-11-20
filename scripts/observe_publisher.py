#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import cv2
import sys, os
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

global odom_x,odom_y,odom_a
global goal_dist, goal_anglei
global observe_data
global goal_x, goal_y

observe_data = np.array([0,0,0,0,0,0,0,0,0,0,0])

odom_x = 0
odom_y = 0
odom_a = 0
goal_x = 0
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

    observe_lines[0] = (middle_line[center])+255*(middle_line[center-1])
    observe_lines[0] = observe_lines[0]/1000.0

    for i in range(4):
        observe_lines[i+1] = middle_line[center-d_width*(i+1)]+255*middle_line[center-d_width*(i+1)-1]
        observe_lines[i+1] = observe_lines[i+1]/1000.0

    for i in range(4):
        ri = 4-i
        observe_lines[i+5] = middle_line[center+d_width*ri]+255*middle_line[center+d_width*ri-1]
        observe_lines[i+5] = observe_lines[i+5]/1000.0

    observe_lines = np.array(observe_lines)

    goal_point = np.array([goal_x, goal_y])
    odom_point = np.array([odom_x, odom_y])
    goal_dist = np.linalg.norm(goal_point-odom_point)

    o_data = []
    o_data = np.append(observe_lines,goal_dist)
    
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

    o_data = np.append(o_data,goal_angle)
    observe_data = o_data

    #np.savetxt('~/my_workspace/src/beginner_tutorials/observe_data.csv',observe_data)
    #np.savetxt(os.path.dirname(__file__) + "/gym_pathplan/envs/observe_data.csv",observe_data)
    #cv2.imshow("test", np_array)
    #cv2.waitKey(1)

def callback_odom(data):
    global odom_x,odom_y,odom_a

    if not data.pose.pose.position.x == data.pose.pose.position.y:
        odom_x = data.pose.pose.position.x
        odom_y = data.pose.pose.position.y
        odom_a = data.pose.pose.orientation.z * (math.pi/2)

def callback_click(data):
    global goal_x, goal_y
    goal_x = data.point.x
    goal_y = data.point.y
    print("=====publish_goal=====")
    print("(" + str(goal_x) + "," + str(goal_y) + ")")

def observe_publish():
    rospy.init_node('observe_publisher', anonymous=True)
    #rospy.Subscriber("camera/depth/image_rect_raw/compressedDepth", CompressedImage, callback)
    #rospy.Subscriber("camera/color/image_raw", Image, callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)
    rospy.Subscriber("/rtabmap/odom",Odometry , callback_odom)
    rospy.Subscriber("/clicked_point", PointStamped, callback_click)

    pub = rospy.Publisher('/observe', Float32MultiArray, queue_size=10)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        pub_array = observe_data
        array_forPublish = Float32MultiArray(data = pub_array)
        pub.publish(array_forPublish)
        #print(observe_data)
        r.sleep()


    rospy.spin()

if __name__ == '__main__':
    observe_publish()
