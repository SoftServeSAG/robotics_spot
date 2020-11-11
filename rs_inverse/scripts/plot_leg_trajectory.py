#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt

from StairsGait import StairsGait
from SpotKinematics import SpotModel
from env_tester import SpotMidlware
time_step = 0.03

spot = SpotModel()
T_bf0 = spot.WorldToFoot

h_raise = 0.168125
w_run = 0.25
k_steps = 10

class Stairs:
    def __init__(self, k_steps, w_run, h_raise, x_stairs_start_shift):
        self.k_steps = k_steps
        self.w_run = w_run
        self.h_raise = h_raise
        self.x_stairs_start_shift = x_stairs_start_shift

    def steps_horizontal(self):
        x = np.zeros((self.k_steps, 2))
        y = np.zeros((self.k_steps, 2))
        x[0, :] = np.array([self.x_stairs_start_shift, self.w_run + self.x_stairs_start_shift])
        y[0, :] = np.array([self.h_raise, self.h_raise])
        for i in range(1, self.k_steps):
            x[i, :] = np.array([x[i-1, 0] + self.w_run, self.w_run + x[i-1, 1]])
            y[i, :] = np.array([y[i-1, 0]+self.h_raise, y[i-1, 1]+self.h_raise])
        return x, y

    def steps_vertical(self):
        x = np.zeros((self.k_steps, 2))
        y = np.zeros((self.k_steps, 2))
        x[0, :] = np.array([self.x_stairs_start_shift, self.x_stairs_start_shift])
        y[0, :] = np.array([0, self.h_raise])
        for i in range(1, self.k_steps):
            x[i, :] = np.array([x[i-1, 0] + self.w_run, self.w_run + x[i-1, 1]])
            y[i, :] = np.array([y[i-1, 0]+self.h_raise, y[i-1, 1]+self.h_raise])
        return x, y

    def plot_stairs(self):
        x_h, y_h = self.steps_horizontal()
        x_v, y_v = self.steps_vertical()
        for i in range(0, self.k_steps):
            plt.plot(x_h[i, :], y_h[i, :], 'k')
            plt.plot(x_v[i, :], y_v[i, :], 'k')
        plt.show()


stairs = Stairs(k_steps=k_steps, w_run=w_run, h_raise=h_raise, x_stairs_start_shift=0)
spot_com = SpotMidlware("spot1", time_step)


import time

x = []
z = []
z_coord_shift = 0
x_coord_shift = 0
t = 0
while t < k_steps:
    start_time = time.time()

    # spot_com.spot_inverse_control()
    spot_com.stairs_gait.time += spot_com.stairs_gait.time_step
    coord = spot_com.stairs_gait.coordinate()
    x.append(coord[0] + x_coord_shift)
    z.append(coord[2] + z_coord_shift)
    elapsed_time = time.time() - start_time
    if elapsed_time < time_step:
        time.sleep(time_step - elapsed_time)
    if coord[2] <= h_raise and abs(coord[0]) > w_run/2:
        # self.Legs_pose[self.legs[self.index_legs]] = coord
        spot_com.stairs_gait.reset()
        x_coord_shift += coord[0] + w_run/2
        z_coord_shift += h_raise
        t += 1



plt.plot(x, z)


stairs.plot_stairs()

plt.show()
