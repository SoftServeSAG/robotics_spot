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

        self.ax01 = subplot2grid((4, 3), (0, 0))
        self.ax02 = subplot2grid((4, 3), (0, 1))
        self.ax03 = subplot2grid((4, 3), (0, 2))
        self.ax04 = subplot2grid((4, 3), (1, 0))
        self.ax05 = subplot2grid((4, 3), (1, 1))
        self.ax06 = subplot2grid((4, 3), (1, 2))
        self.ax07 = subplot2grid((4, 3), (2, 0))
        self.ax08 = subplot2grid((4, 3), (2, 1))
        self.ax09 = subplot2grid((4, 3), (2, 2))
        self.ax10 = subplot2grid((4, 3), (3, 0))
        self.ax11 = subplot2grid((4, 3), (3, 1))
        self.ax12 = subplot2grid((4, 3), (3, 2))


        # Set titles of subplots
        self.ax01.set_title('hip_x')
        self.ax02.set_title('hip_y')
        self.ax03.set_title('knee')

        # Set titles of subplots
        self.ax01.set_ylabel('Front Left Leg')
        self.ax04.set_ylabel('Front Right Leg')
        self.ax07.set_ylabel('Rear Left Leg')
        self.ax10.set_ylabel('Rear Right Leg')

        self.y_flhx_state = []
        self.y_frhx_state = []
        self.y_rlhx_state = []
        self.y_rrhx_state = []
        self.y_flhy_state = []
        self.y_frhy_state = []
        self.y_rlhy_state = []
        self.y_rrhy_state = []
        self.y_flk_state = []
        self.y_frk_state = []
        self.y_rlk_state = []
        self.y_rrk_state = []

        self.x_flhx_state, self.yy_flhx_state = np.array, np.array
        self.x_flhy_state, self.yy_flhy_state = np.array, np.array
        self.x_flk_state, self.yy_flk_state = np.array, np.array

        self.x_frhx_state, self.yy_frhx_state = np.array, np.array
        self.x_frhy_state, self.yy_frhy_state = np.array, np.array
        self.x_frk_state, self.yy_frk_state = np.array, np.array

        self.x_rlhx_state, self.yy_rlhx_state = np.array, np.array
        self.x_rlhy_state, self.yy_rlhy_state = np.array, np.array
        self.x_rlk_state, self.yy_rlk_state = np.array, np.array

        self.x_rrhx_state, self.yy_rrhx_state = np.array, np.array
        self.x_rrhy_state, self.yy_rrhy_state = np.array, np.array
        self.x_rrk_state, self.yy_rrk_state = np.array, np.array

        self.k_flhx_state = 0
        self.k_frhx_state = 0
        self.k_rlhx_state = 0
        self.k_rrhx_state = 0
        self.k_flhy_state = 0
        self.k_frhy_state = 0
        self.k_rlhy_state = 0
        self.k_rrhy_state = 0
        self.k_flk_state = 0
        self.k_frk_state = 0
        self.k_rlk_state = 0
        self.k_rrk_state = 0

        self.x = []

    def plot_init(self):
        self.ax01.set_xlim(0, self.x_lim)
        self.ax01.set_ylim(-1.5, 1.5)
        self.ax02.set_xlim(0, self.x_lim)
        self.ax02.set_ylim(-1.5, 3)
        self.ax03.set_xlim(0, self.x_lim)
        self.ax03.set_ylim(-3, 3)
        self.ax04.set_xlim(0, self.x_lim)
        self.ax04.set_ylim(-1.5, 1.5)
        self.ax05.set_xlim(0, self.x_lim)
        self.ax05.set_ylim(-1.5, 3)
        self.ax06.set_xlim(0, self.x_lim)
        self.ax06.set_ylim(-3, 3)
        self.ax07.set_xlim(0, self.x_lim)
        self.ax07.set_ylim(-1.5, 1.5)
        self.ax08.set_xlim(0, self.x_lim)
        self.ax08.set_ylim(-1.5, 3)
        self.ax09.set_xlim(0, self.x_lim)
        self.ax09.set_ylim(-3, 3)
        self.ax10.set_xlim(0, self.x_lim)
        self.ax10.set_ylim(-1.5, 1.5)
        self.ax11.set_xlim(0, self.x_lim)
        self.ax11.set_ylim(-1.5, 3)
        self.ax12.set_xlim(0, self.x_lim)
        self.ax12.set_ylim(-3, 3)
        self.ax01.grid(True)
        self.ax02.grid(True)
        self.ax03.grid(True)
        self.ax04.grid(True)
        self.ax05.grid(True)
        self.ax06.grid(True)
        self.ax07.grid(True)
        self.ax08.grid(True)
        self.ax09.grid(True)
        self.ax10.grid(True)
        self.ax11.grid(True)
        self.ax12.grid(True)

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

    def callback_front_right_hip_x(self, data):
        if len(self.y_frhx_state) > self.max_length:
            del self.y_frhx_state[0:self.short_length]
            self.k_frhx_state += 1
        self.y_frhx_state.append(data.process_value)

    def callback_front_right_hip_y(self, data):
        if len(self.y_frhy_state) > self.max_length:
            del self.y_frhy_state[0:self.short_length]
            self.k_frhy_state += 1
        self.y_frhy_state.append(data.process_value)

    def callback_front_right_knee(self, data):
        if len(self.y_frk_state) > self.max_length:
            del self.y_frk_state[0:self.short_length]
            self.k_frk_state += 1
        self.y_frk_state.append(data.process_value)

    def callback_rear_left_hip_x(self, data):
        if len(self.y_rlhx_state) > self.max_length:
            del self.y_rlhx_state[0:self.short_length]
            self.k_rlhx_state += 1
        self.y_rlhx_state.append(data.process_value)

    def callback_rear_left_hip_y(self, data):
        if len(self.y_rlhy_state) > self.max_length:
            del self.y_rlhy_state[0:self.short_length]
            self.k_rlhy_state += 1
        self.y_rlhy_state.append(data.process_value)

    def callback_rear_left_knee(self, data):
        if len(self.y_rlk_state) > self.max_length:
            del self.y_rlk_state[0:self.short_length]
            self.k_rlk_state += 1
        self.y_rlk_state.append(data.process_value)

    def callback_rear_right_hip_x(self, data):
        if len(self.y_rrhx_state) > self.max_length:
            del self.y_rrhx_state[0:self.short_length]
            self.k_rrhx_state += 1
        self.y_rrhx_state.append(data.process_value)

    def callback_rear_right_hip_y(self, data):
        if len(self.y_rrhy_state) > self.max_length:
            del self.y_rrhy_state[0:self.short_length]
            self.k_rrhy_state += 1
        self.y_rrhy_state.append(data.process_value)

    def callback_rear_right_knee(self, data):
        if len(self.y_rrk_state) > self.max_length:
            del self.y_rrk_state[0:self.short_length]
            self.k_rrk_state += 1
        self.y_rrk_state.append(data.process_value)

    def transform_data(self, y, k):
        yy = np.array(y)
        n = np.size(yy)
        xx = np.arange(k * self.short_length, n + k * self.short_length)
        x_min = k * self.short_length
        x_max = n + k * self.short_length + self.x_lim
        return xx, yy, x_min, x_max

    def update_plot(self, i):
        spot_name = str(input("Tell me spot name: "))
        rospy.init_node(spot_name + 'spot_plot_visual_node')
        rospy.Subscriber("/" + spot_name + "/spot/joint_front_left_hip_x_controller/state", JointControllerState,
                         vis.callback_front_left_hip_x)
        rospy.Subscriber("/" + spot_name + "/joint_front_left_hip_y_controller/state", JointControllerState,
                         vis.callback_front_left_hip_y)
        rospy.Subscriber("/" + spot_name + "/joint_front_left_knee_controller/state", JointControllerState,
                         vis.callback_front_left_knee)

        rospy.Subscriber("/" + spot_name + "/joint_front_right_hip_x_controller/state", JointControllerState,
                         vis.callback_front_right_hip_x)
        rospy.Subscriber("/" + spot_name + "/joint_front_right_hip_y_controller/state", JointControllerState,
                         vis.callback_front_right_hip_y)
        rospy.Subscriber("/" + spot_name + "/joint_front_right_knee_controller/state", JointControllerState,
                         vis.callback_front_right_knee)

        rospy.Subscriber("/" + spot_name + "/joint_rear_left_hip_x_controller/state", JointControllerState,
                         vis.callback_rear_left_hip_x)
        rospy.Subscriber("/" + spot_name + "/joint_rear_left_hip_y_controller/state", JointControllerState,
                         vis.callback_rear_left_hip_y)
        rospy.Subscriber("/" + spot_name + "/joint_rear_left_knee_controller/state", JointControllerState,
                         vis.callback_rear_left_knee)

        rospy.Subscriber("/" + spot_name + "/joint_rear_right_hip_x_controller/state", JointControllerState,
                         vis.callback_rear_right_hip_x)
        rospy.Subscriber("/" + spot_name + "/joint_rear_right_hip_y_controller/state", JointControllerState,
                         vis.callback_rear_right_hip_y)
        rospy.Subscriber("/" + spot_name + "/joint_rear_right_knee_controller/state", JointControllerState,
                         vis.callback_rear_right_knee)
        # ---- Front Left
        # front left hip_x
        self.x_flhx_state, self.yy_flhx_state, x_min_flhx_state, x_max_flhx_state = self.transform_data(
            self.y_flhx_state, self.k_flhx_state)
        self.ax01.set_xlim(x_min_flhx_state, x_max_flhx_state)
        self.ax01.plot(self.x_flhx_state, self.yy_flhx_state, 'b-', ms=1, label="g")
        # front left hip_y
        self.x_flhy_state, self.yy_flhy_state, x_min_flhy_state, x_max_flhy_state = self.transform_data(
            self.y_flhy_state, self.k_flhy_state)
        self.ax02.set_xlim(x_min_flhy_state, x_max_flhy_state)
        self.ax02.plot(self.x_flhy_state, self.yy_flhy_state, 'b-', ms=1, label="g")
        # front left knee
        self.x_flk_state, self.yy_flk_state, x_min_flk_state, x_max_flk_state = self.transform_data(
            self.y_flk_state, self.k_flk_state)
        self.ax03.set_xlim(x_min_flk_state, x_max_flk_state)
        self.ax03.plot(self.x_flk_state, self.yy_flk_state, 'b-', ms=1, label="g")
        # ---- Front Right
        # front right hip_x
        self.x_frhx_state, self.yy_frhx_state, x_min_frhx_state, x_max_frhx_state = self.transform_data(
            self.y_frhx_state, self.k_frhx_state)
        self.ax04.set_xlim(x_min_frhx_state, x_max_frhx_state)
        self.ax04.plot(self.x_frhx_state, self.yy_frhx_state, 'b-', ms=1, label="g")
        # front right hip_y
        self.x_frhy_state, self.yy_frhy_state, x_min_frhy_state, x_max_frhy_state = self.transform_data(
            self.y_frhy_state, self.k_frhy_state)
        self.ax05.set_xlim(x_min_frhy_state, x_max_frhy_state)
        self.ax05.plot(self.x_frhy_state, self.yy_frhy_state, 'b-', ms=1, label="g")
        # front right knee
        self.x_frk_state, self.yy_frk_state, x_min_frk_state, x_max_frk_state = self.transform_data(
            self.y_frk_state, self.k_frk_state)
        self.ax06.set_xlim(x_min_frk_state, x_max_frk_state)
        self.ax06.plot(self.x_frk_state, self.yy_frk_state, 'b-', ms=1, label="g")

        # ---- Rear Left
        # rear left hip_x
        self.x_rlhx_state, self.yy_rlhx_state, x_min_rlhx_state, x_max_rlhx_state = self.transform_data(
            self.y_rlhx_state, self.k_rlhx_state)
        self.ax07.set_xlim(x_min_rlhx_state, x_max_rlhx_state)
        self.ax07.plot(self.x_rlhx_state, self.yy_rlhx_state, 'b-', ms=1, label="g")
        # rear left hip_y
        self.x_rlhy_state, self.yy_rlhy_state, x_min_rlhy_state, x_max_rlhy_state = self.transform_data(
            self.y_rlhy_state, self.k_rlhy_state)
        self.ax08.set_xlim(x_min_rlhy_state, x_max_rlhy_state)
        self.ax08.plot(self.x_rlhy_state, self.yy_rlhy_state, 'b-', ms=1, label="g")
        # rear left knee
        self.x_rlk_state, self.yy_rlk_state, x_min_rlk_state, x_max_rlk_state = self.transform_data(
            self.y_rlk_state, self.k_rlk_state)
        self.ax09.set_xlim(x_min_rlk_state, x_max_rlk_state)
        self.ax09.plot(self.x_rlk_state, self.yy_rlk_state, 'b-', ms=1, label="g")
        # ---- Rear Right
        # rear right hip_x
        self.x_rrhx_state, self.yy_rrhx_state, x_min_rrhx_state, x_max_rrhx_state = self.transform_data(
            self.y_rrhx_state, self.k_rrhx_state)
        self.ax10.set_xlim(x_min_rrhx_state, x_max_rrhx_state)
        self.ax10.plot(self.x_rrhx_state, self.yy_rrhx_state, 'b-', ms=1, label="g")
        # rear right hip_y
        self.x_rrhy_state, self.yy_rrhy_state, x_min_rrhy_state, x_max_rrhy_state = self.transform_data(
            self.y_rrhy_state, self.k_rrhy_state)
        self.ax11.set_xlim(x_min_rrhy_state, x_max_rrhy_state)
        self.ax11.plot(self.x_rrhy_state, self.yy_rrhy_state, 'b-', ms=1, label="g")
        # rear right knee
        self.x_rrk_state, self.yy_rrk_state, x_min_rrk_state, x_max_rrk_state = self.transform_data(
            self.y_rrk_state, self.k_rrk_state)
        self.ax12.set_xlim(x_min_rrk_state, x_max_rrk_state)
        self.ax12.plot(self.x_rrk_state, self.yy_rrk_state, 'b-', ms=1, label="g")


vis = Visualiser()

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show()
