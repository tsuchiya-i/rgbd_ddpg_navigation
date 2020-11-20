#coding:utf-8

import numpy as np
import matplotlib.pyplot as plt
import math,time

def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0
    return angle

def calc_angleid(angle, yaw_reso):
    if angle < -yaw_reso/2:
        angle += math.pi * 2.0
    trans_angle = angle + yaw_reso/2
    angleid = math.floor(trans_angle/yaw_reso)
    return angleid

class lidarinfo:
    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.angleid = 0
        self.init = True
    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle) + "," + str(self.angleid) + "," + str(self.init)

class raycast(object):
    def __init__(self, pose, grid_map, grid_height, grid_width, xyreso, yawreso, min_range, max_range):
        self.pose      = pose
        self.grid_map  = grid_map
        self.grid_height = grid_height
        self.grid_width = grid_width
        self.xyreso    = xyreso
        self.yawreso   = yawreso
        self.min_range = min_range
        self.max_range = max_range

    def calc_obstacle_position(self):
        sxx = int(self.pose[0]/self.xyreso) - int(self.max_range/self.xyreso)
        fxx = int(self.pose[0]/self.xyreso) + int(self.max_range/self.xyreso)
        syy = (self.grid_height-1)-(int(self.pose[1]/self.xyreso)+int(self.max_range/self.xyreso))
        fyy = (self.grid_height-1)-(int(self.pose[1]/self.xyreso)-int(self.max_range/self.xyreso))
        if sxx<0:
            sxx = 0
        if fxx>self.grid_width-1:
            fxx = self.grid_width-1
        if syy<0:
            syy = 0
        if fyy>self.grid_height-1:
            fyy = self.grid_height-1
        """
        print("sx = "+str(sxx))
        print("fx = "+str(fxx))
        print("sy = "+str(syy))
        print("fy = "+str(fyy))
        """


        obstacle_grid = np.where(0<self.grid_map[syy:fyy,sxx:fxx])
        obstacle_position = np.zeros((len(obstacle_grid[0]), 2))
        for i, (ix, iy) in enumerate(zip(obstacle_grid[1], obstacle_grid[0])):#fix
            x = (ix+sxx)*self.xyreso
            y = (self.grid_height-(iy+syy))*self.xyreso
            obstacle_position[i][0] = x
            obstacle_position[i][1] = y
        #print(obstacle_position)
        #print(len(obstacle_position))
        return obstacle_position

    """
    def calc_obstacle_position(self):
        izero = 0
        si = int(self.pose[0]/self.xyreso)+int(self.max_range/self.xyreso)
        if si < izero:
            si = izero
        fi = int(self.pose[0]/self.xyreso)-int(self.max_range/self.xyreso)
        if fi > self.grid_width-1:
            fi = self.grid_width-1
        sj = int(self.pose[1]/self.xyreso)-int(self.max_range/self.xyreso)
        if sj < izero:
            sj = izero
        fj = int(self.pose[1]/self.xyreso)+int(self.max_range/self.xyreso)
        if fj > self.grid_height-1:
            fj = self.grid_height-1

        obstacle_grid = np.where(0<self.grid_map[(self.grid_height-1)-fj:(self.grid_height-1)-sj,si:fi])
        obstacle_position = np.zeros((len(obstacle_grid[0]), 2))
        #print(self.grid_map)
        #print(self.grid_map[(self.grid_height-1)-fj:(self.grid_height-1)-sj,si:fi])
        #print(len(obstacle_grid[0]))
        for i, (ix, iy) in enumerate(zip(obstacle_grid[1], obstacle_grid[0])):#fix
            x = (ix+sj)*self.xyreso
            y = (iy+si)*self.xyreso
            obstacle_position[i][0] = x
            obstacle_position[i][1] = y
        #print(obstacle_position)
        #print(len(obstacle_position))
        return obstacle_position
    """
    def calc_obstacle_position__(self):
        sstime = time.time()
        rm_i=[]
        obstacle_grid = np.where(0<self.grid_map)
        obstacle_position = np.zeros((len(obstacle_grid[0]), 2))
        for i, (ix, iy) in enumerate(zip(obstacle_grid[1], obstacle_grid[0])):#fix
            x = ix*self.xyreso
            y = iy*self.xyreso
            if  self.max_range < abs(self.pose[0]-x) or self.max_range < abs(self.pose[1]-y):
                rm_i.insert(0,i)
            obstacle_position[i][0] = x
            obstacle_position[i][1] = y
        #print(obstacle_position)
        for i in rm_i:#遠い障害物は削除
            obstacle_position = np.delete(obstacle_position, i, 0)
        #print(obstacle_position)
        cctime = time.time()-sstime
        #print('{:.5f}'.format(cctime))
        return obstacle_position

    def transform(self, x, y, obstacle_position):
        transform_position = np.zeros((obstacle_position.shape))
        for i in range(obstacle_position.shape[0]):
            transform_position[i][0] = obstacle_position[i][0]-x
            transform_position[i][1] = obstacle_position[i][1]-y
        return transform_position

    def rotation(self, radian, obstacle_position):
        rotation = np.array([[math.cos(radian),   math.sin(radian)],
                             [-math.sin(radian),  math.cos(radian)]]);
        rotation_position = np.zeros(obstacle_position.shape)
        for i in range(obstacle_position.shape[0]):
            rotation_position[i] = np.dot(rotation, obstacle_position[i])
        return rotation_position

    def raycasting(self):
        sstime = time.time()
        obstacle_position  = self.calc_obstacle_position()
        transform_position = self.transform(self.pose[0], self.pose[1], obstacle_position)
        rotation_position = self.rotation(self.pose[2], transform_position)

        lidar_num = int(round((math.pi * 2.0) / self.yawreso) + 1)

        lidar_array = [[] for i in range(lidar_num)]
        for i in range(lidar_num):
            lidar = lidarinfo()
            lidar.angle = i*self.yawreso
            lidar.angleid = i
            if lidar.angle < math.pi/4.0 or 2.0*math.pi > lidar.angle > 2.0*math.pi-math.pi/4.0: #何もないときはmaxを代入
                lidar.px = self.max_range*math.cos(lidar.angle)
                lidar.py = self.max_range*math.sin(lidar.angle)
                lidar.d = self.max_range
                lidar.init = None

            lidar_array[i] = lidar

        for pose in rotation_position:
            x = pose[0]
            y = pose[1]
            d = np.sqrt(x**2+y**2)
            angle = math.atan2(y, x)

            if(self.max_range<d):
                d = self.max_range
            elif(d<self.min_range):
                d = self.min_range

            if abs(angle) < math.pi/4.0:#LiDAR測定角指定
                angleid = calc_angleid(angle, self.yawreso)

                if lidar_array[int(angleid)].init:
                    lidar_array[int(angleid)].px = x
                    lidar_array[int(angleid)].py = y
                    lidar_array[int(angleid)].d  = d
                    lidar_array[int(angleid)].init = False
            
                elif d < lidar_array[int(angleid)].d:
                    lidar_array[int(angleid)].px = x
                    lidar_array[int(angleid)].py = y
                    lidar_array[int(angleid)].d  = d

        raycast_map = []

        for lidar in lidar_array:
            angle = lidar.angle
            if not lidar.init:
                x = lidar.d*math.cos(angle)
                y = lidar.d*math.sin(angle)
                raycast_map.append([x, y, angle, lidar.d, int(lidar.angleid)])
        raycast_array = np.array(raycast_map)

        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.plot(obstacle_position.T[0], obstacle_position.T[1], "og")
        # plt.plot(self.pose[0], self.pose[1], "ob")
        # plt.show()

        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.plot(transform_position.T[0], transform_position.T[1], "og")
        # plt.plot(0, 0, "ob")
        # plt.show()

        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.plot(rotation_position.T[0], rotation_position.T[1], "og")
        # plt.plot(0, 0, "ob")
        # plt.show()

        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.plot(rotation_position.T[0], rotation_position.T[1], "og")
        # plt.plot(0, 0, "ob")
        # for x, y in zip(raycast_array.T[0], raycast_array.T[1]):
        #    plt.plot([0.0, x], [0.0, y], 'c-')
        # plt.show()
        
        cctime = time.time()-sstime
        #print('{:.5f}'.format(cctime))
        return raycast_array
