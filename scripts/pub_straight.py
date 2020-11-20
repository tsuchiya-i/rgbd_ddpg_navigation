#!/usr/bin/env python
import rospy
import numpy as np
import gym, os
import gym_pathplan
from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam
from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess
import matplotlib.pyplot as plt
import tensorflow as tf
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

global observe_data
observe_data = np.array([0,0,0,0,0,0,0,0,0,0,0])

def callback_observe(data):
    global observe_data
    observe_data = np.array(data.data)
    #print(observe_data)
    #print(agent.forward(observe_data))
    

rospy.init_node('pub_drive', anonymous=True)
rospy.Subscriber("/observe", Float32MultiArray, callback_observe)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10) # 5hz
while not rospy.is_shutdown():
    pub_vel = Twist()
    pub_vel.linear.x = 0.6
    pub_vel.angular.z = -0.05 
    pub.publish(pub_vel)
    print(str(pub_vel))
    r.sleep()

rospy.spin()
