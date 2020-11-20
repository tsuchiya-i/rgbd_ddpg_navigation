#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

global observe_data
observe_data = np.array([0,0,0,0,0,0,0,0,0,0,0])

def callback_observe(data):
    global observe_data
    observe_data = np.array(data.data)
    #print(observe_data)
    #print(agent.forward(observe_data))
    

rospy.init_node('pub_goal', anonymous=True)
pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
r = rospy.Rate(10) # 5hz
while not rospy.is_shutdown():
    pub_goal = PointStamped()
    pub_goal.header.frame_id ="map"
    pub_goal.point.x =10.5 
    pub_goal.point.y =-17 
    pub_goal.point.x =20.5 
    pub_goal.point.y =-30 
    pub.publish(pub_goal)
    print(str(pub_goal))
    r.sleep()

rospy.spin()
