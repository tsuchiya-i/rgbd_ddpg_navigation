#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time


global observe_data
observe_data = np.array([0,0,0,0,0,0,0,0,0,0,0])


fig, ax = plt.subplots(1, 1)
ax.set_ylim(0, 10)
xx = []
yy = []
global step_count
step_count = 0

def callback_observe(data):
    global observe_data
    global step_count
    global xx,yy
    x = []
    y = []
    observe_data = np.array(data.data)
    lidar = observe_data[0:9]

    step_count += 1
    for i in range(5):
        for j in range(10):
            y.append(lidar[4-i])
            x.append(i*10+j)
    for i in range(4):
        for j in range(10):
            y.append(lidar[8-i])
            x.append((i+5)*10+j)

    #print(len(y))
    xx = x
    yy = y
    
rospy.init_node('observe_viewer', anonymous=True)
rospy.Subscriber("/observe", Float32MultiArray, callback_observe)

while not rospy.is_shutdown():
    plt.xlim(0,100)
    line, = ax.plot(xx, yy, color='blue')
    plt.pause(0.01)
    line.remove()
    time.sleep(0.02)


rospy.spin()
