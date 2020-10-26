#!/usr/bin/env python
import rospy
from control_msgs.msg import JointControllerState
from matplotlib.pylab import *
import numpy as np
from matplotlib.animation import FuncAnimation
import time



class Visualiser:
    def __init__(self):
        self.fig = figure(num=0, figsize=(12, 8))
        self.fig.suptitle("Joints angels", fontsize=12)

        self.max_length = 10000
        self.short_length = 5000

        self.x_lim = 1000

        self.ax01 = subplot2grid((3, 1), (0, 0))
        self.ax02 = subplot2grid((3, 1), (1, 0))
        self.ax03 = subplot2grid((3, 1), (2, 0))


        # Set titles of subplots
        self.ax01.set_title('Hip revolution x-axis')
        self.ax02.set_title('Hip revolution y-axis')
        self.ax03.set_title('Knee revolution')


        # Set titles of subplots
        # self.ax01.set_title('Front left hip_x')
        # self.ax02.set_title('Front left hip_y')
        # self.ax03.set_title('Front left knee')
        # self.ax04.set_title('Front right hip_x')
        # self.ax05.set_title('Front right hip_y')
        # self.ax06.set_title('Front right knee')
        # self.ax07.set_title('Rear left hip_x')
        # self.ax08.set_title('Rear left hip_y')
        # self.ax09.set_title('Rear left knee')
        # self.ax10.set_title('Rear right hip_x')
        # self.ax11.set_title('Rear right hip_y')
        # self.ax12.set_title('Rear right knee')

        # self.x_flhx_command, self.y_flhx_command = [], []
        # self.x_frhx_command, self.y_frhx_command = [], []
        # self.x_rlhx_command, self.y_rlhx_command = [], []
        # self.x_rrhx_command, self.y_rrhx_command = [], []
        # self.x_flhy_command, self.y_flhy_command = [], []
        # self.x_frhy_command, self.y_frhy_command = [], []
        # self.x_rlhy_command, self.y_rlhy_command = [], []
        # self.x_rrhy_command, self.y_rrhy_command = [], []
        # self.x_flk_command, self.y_flk_command = [], []
        # self.x_frk_command, self.y_frk_command = [], []
        # self.x_rlk_command, self.y_rlk_command = [], []
        # self.x_rrk_command, self.y_rrk_command = [], []

        self.y_flhx_state = []
        self.y_flhy_state = []
        self.y_flk_state = []

        self.y_flhx_set_point = []
        self.y_flhy_set_point = []
        self.y_flk_set_point = []

        self.x_flhx_state, self.yy_flhx_state = np.array, np.array
        self.x_flhy_state, self.yy_flhy_state = np.array, np.array
        self.x_flk_state, self.yy_flk_state = np.array, np.array\

        self.x_flhx_set_point, self.yy_flhx_set_point = np.array, np.array
        self.x_flhy_set_point, self.yy_flhy_set_point = np.array, np.array
        self.x_flk_set_point, self.yy_flk_set_point = np.array, np.array

        self.k_flhx_state = 0
        self.k_flhy_state = 0
        self.k_flk_state = 0

        self.k_flhx_set_point = 0
        self.k_flhy_set_point = 0
        self.k_flk_set_point = 0

    def plot_init(self):
        self.ax01.set_xlim(0, self.x_lim)
        self.ax01.set_ylim(-1.5, 1.5)
        self.ax02.set_xlim(0, self.x_lim)
        self.ax02.set_ylim(-1.5, 3)
        self.ax03.set_xlim(0, self.x_lim)
        self.ax03.set_ylim(-3, 3)
        self.ax01.grid(True)
        self.ax02.grid(True)
        self.ax03.grid(True)

        self.ax01.plot([], [], 'b-', ms=1, label="state")
        self.ax02.plot([], [], 'b-', ms=1, label="state")
        self.ax03.plot([], [], 'b-', ms=1, label="state")
        self.ax01.plot([], [], 'r-', ms=0.4, label="set point")
        self.ax02.plot([], [], 'r-', ms=0.4, label="set point")
        self.ax03.plot([], [], 'r-', ms=0.4, label="set point")
        self.ax01.legend()
        self.ax02.legend()
        self.ax03.legend()

    def callback_front_left_hip_x(self, data):
        if len(self.y_flhx_state) > self.max_length:
            del self.y_flhx_state[0:self.short_length]
            self.k_flhx_state += 1
        self.y_flhx_state.append(data.process_value)

    def callback_front_left_hip_y(self, data):
        if len(self.y_flhy_state) > self.max_length:
            del self.y_flhy_state[0:self.short_length]
            self.k_flhy_state += 1
        self.y_flhy_state.append(data.process_value)

    def callback_front_left_knee(self, data):
        if len(self.y_flk_state) > self.max_length:
            del self.y_flk_state[0:self.short_length]
            self.k_flk_state += 1
        self.y_flk_state.append(data.process_value)

    # Set point callback
    def callback_front_left_hip_x_set_point(self, data):
        if len(self.y_flhx_set_point) > self.max_length:
            del self.y_flhx_set_point[0:self.short_length]
            self.k_flhx_set_point += 1
        self.y_flhx_set_point.append(data.set_point)

    def callback_front_left_hip_y_set_point(self, data):
        if len(self.y_flhy_set_point) > self.max_length:
            del self.y_flhy_set_point[0:self.short_length]
            self.k_flhy_set_point += 1
        self.y_flhy_set_point.append(data.set_point)

    def callback_front_left_knee_set_point(self, data):
        if len(self.y_flk_set_point) > self.max_length:
            del self.y_flk_set_point[0:self.short_length]
            self.k_flk_set_point += 1
        self.y_flk_set_point.append(data.set_point)

    def transform_data(self, y, k):
        yy = np.array(y)
        n = np.size(yy)
        xx = np.arange(k * self.short_length, n + k * self.short_length)
        x_min = k * self.short_length
        x_max = n + k * self.short_length + self.x_lim
        return xx, yy, x_min, x_max

    def update_plot(self, i):
        rospy.init_node('spot_plot_visual_node')
        rospy.Subscriber("/spot/joint_front_left_hip_x_controller/state", JointControllerState,
                         vis.callback_front_left_hip_x)
        rospy.Subscriber("/spot/joint_front_left_hip_y_controller/state", JointControllerState,
                         vis.callback_front_left_hip_y)
        rospy.Subscriber("/spot/joint_front_left_knee_controller/state", JointControllerState,
                         vis.callback_front_left_knee)

        rospy.Subscriber("/spot/joint_front_left_hip_x_controller/state", JointControllerState,
                         vis.callback_front_left_hip_x_set_point)
        rospy.Subscriber("/spot/joint_front_left_hip_y_controller/state", JointControllerState,
                         vis.callback_front_left_hip_y_set_point)
        rospy.Subscriber("/spot/joint_front_left_knee_controller/state", JointControllerState,
                         vis.callback_front_left_knee_set_point)

        # ---- Front Left
        # ----States
        # front left hip_x
        self.x_flhx_state, self.yy_flhx_state, x_min_flhx_state, x_max_flhx_state = self.transform_data(
            self.y_flhx_state, self.k_flhx_state)
        self.ax01.set_xlim(x_min_flhx_state, x_max_flhx_state)
        self.ax01.plot(self.x_flhx_state, self.yy_flhx_state, 'b-', ms=1, label="state")
        # front left hip_y
        self.x_flhy_state, self.yy_flhy_state, x_min_flhy_state, x_max_flhy_state = self.transform_data(
            self.y_flhy_state, self.k_flhy_state)
        self.ax02.set_xlim(x_min_flhy_state, x_max_flhy_state)
        self.ax02.plot(self.x_flhy_state, self.yy_flhy_state, 'b-', ms=1, label="state")
        # front left knee
        self.x_flk_state, self.yy_flk_state, x_min_flk_state, x_max_flk_state = self.transform_data(
            self.y_flk_state, self.k_flk_state)
        self.ax03.set_xlim(x_min_flk_state, x_max_flk_state)
        self.ax03.plot(self.x_flk_state, self.yy_flk_state, 'b-', ms=1, label="state")
        # ----Set Points
        self.x_flhx_set_point, self.yy_flhx_set_point, x_min_flhx_set_point, x_max_flhx_set_point = self.transform_data(
            self.y_flhx_set_point, self.k_flhx_set_point)
        self.ax01.set_xlim(x_min_flhx_set_point, x_max_flhx_set_point)
        self.ax01.plot(self.x_flhx_set_point, self.yy_flhx_set_point, 'r-', ms=0.4, label="set point")
        # front left hip_y
        self.x_flhy_set_point, self.yy_flhy_set_point, x_min_flhy_set_point, x_max_flhy_set_point = self.transform_data(
            self.y_flhy_set_point, self.k_flhy_set_point)
        self.ax02.set_xlim(x_min_flhy_set_point, x_max_flhy_set_point)
        self.ax02.plot(self.x_flhy_set_point, self.yy_flhy_set_point, 'r-', ms=0.4, label="set point")
        # front left knee
        self.x_flk_set_point, self.yy_flk_set_point, x_min_flk_set_point, x_max_flk_set_point = self.transform_data(
            self.y_flk_set_point, self.k_flk_set_point)
        self.ax03.set_xlim(x_min_flk_set_point, x_max_flk_set_point)
        self.ax03.plot(self.x_flk_set_point, self.yy_flk_set_point, 'r-', ms=0.4, label="set point")

vis = Visualiser()

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show()
