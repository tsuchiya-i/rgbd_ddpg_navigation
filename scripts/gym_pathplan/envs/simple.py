#coding:utf-8

import numpy as np
import math
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import sys
import os
import cv2,time
import random

class Simple(gym.Env):
    metadata = {'render.modes' : ['human', 'rgb_array']}

    def __init__(self):
        self.dt = 0.1
        self.state = np.array([9, 10, math.radians(90), 0.0, 0.0])

        self.max_dist = 200 #最大の目標地点までの距離[m]

        # robot param
        self.robot_radius = 0.2 #[m]

        # action param
        self.max_velocity = 0.8   # [m/s]
        self.min_velocity = -0.4  # [m/s]
        self.max_velocity_acceleration = 0.2  # [m/ss]
        self.min_velocity_acceleration = -0.2 # [m/ss]
        self.min_angular_velocity = math.radians(-40)  # [rad/s]
        self.max_angular_velocity = math.radians(40) # [rad/s]
        self.min_angular_acceleration = math.radians(-40)  # [rad/ss]
        self.max_angular_acceleration = math.radians(40) # [rad/ss]

        # lidar param
        self.yawreso = math.radians(10) # [rad]
        self.min_range = 0.20 # [m]
        self.max_range = 10.0 # [m]
        self.lidar_num = int(round(math.radians(360)/self.yawreso)+1)

        # set action_space (velocity[m/s], omega[rad/s])
        self.action_low  = np.array([self.min_velocity, self.min_angular_velocity]) 
        self.action_high = np.array([self.max_velocity, self.max_angular_velocity]) 
        self.action_space = spaces.Box(self.action_low, self.action_high, dtype=np.float32)

        # set observation_space
        self.min_yawrate  = math.radians(0)  # [rad]
        self.max_yawrate  = math.radians(360) # [rad]
        # state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.state_low  = np.array([0.0, 0.0, self.min_yawrate, self.min_velocity, self.min_angular_velocity])
        self.state_high = np.array([0.0, 0.0, self.max_yawrate, self.max_velocity, self.max_angular_velocity])

        self.observation_low = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi])
        self.observation_high = np.array([self.max_range,self.max_range,self.max_range,self.max_range,self.max_range,self.max_range,self.max_range,self.max_range,self.max_range,self.max_dist,math.pi])
        self.observation_space = spaces.Box(low = self.observation_low, high = self.observation_high, dtype=np.float32)

        self.viewer = None
        self.vis_lidar = True

    
    # 状態を初期化し、初期の観測値を返す
    def reset(self):
        self.distgoal = np.array([10, 0]) #[d, anglegoal]
        self.observation = self.observe()
        self.done = False
        # world_time reset
        return self.observation
    
    # actionを実行し、結果を返す
    def step(self, action):
        self.observation = self.observe()
        reward = 0.0
        self.done = self.is_done(True)
        return self.observation, reward, self.done, {}

    # ゴールに到達したかを判定
    def is_goal(self, show=False):
        if False:
            if show:
                print("Goal")
            return True
        else:
            return False

    # 報酬値を返す
    def reward(self):
        return 0.0

    # 終端状態か確認
    def is_done(self, show=False):
        return self.is_goal(show)

    # 観測結果を表示
    def observe(self):
        try:
            observe_data = np.loadtxt(os.path.dirname(__file__) + "/observe_data.csv")
            #print(observe_data)
            observation = observe_data
        except:
            print("not fou")
            observation = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,10.0,1.0])

        return observation
