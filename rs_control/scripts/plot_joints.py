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
        self.ax01.set_title('Front left hip_x')
        self.ax02.set_title('Front left hip_y')
        self.ax03.set_title('Front left knee')
        self.ax04.set_title('Front right hip_x')
        self.ax05.set_title('Front right hip_y')
        self.ax06.set_title('Front right knee')
        self.ax07.set_title('Rear left hip_x')
        self.ax08.set_title('Rear left hip_y')
        self.ax09.set_title('Rear left knee')
        self.ax10.set_title('Rear right hip_x')
        self.ax11.set_title('Rear right hip_y')
        self.ax12.set_title('Rear right knee')

        self.p011, = self.ax01.plot([], [], 'b-', ms=1, label="g")
        self.p021, = self.ax02.plot([], [], 'b-', ms=1, label="g")
        self.p031, = self.ax03.plot([], [], 'b-', ms=1, label="g")
        self.p041, = self.ax04.plot([], [], 'b-', ms=1, label="g")
        self.p051, = self.ax05.plot([], [], 'b-', ms=1, label="g")
        self.p061, = self.ax06.plot([], [], 'b-', ms=1, label="g")
        self.p071, = self.ax07.plot([], [], 'b-', ms=1, label="g")
        self.p081, = self.ax08.plot([], [], 'b-', ms=1, label="g")
        self.p091, = self.ax09.plot([], [], 'b-', ms=1, label="g")
        self.p101, = self.ax10.plot([], [], 'b-', ms=1, label="g")
        self.p111, = self.ax11.plot([], [], 'b-', ms=1, label="g")
        self.p121, = self.ax12.plot([], [], 'b-', ms=1, label="g")

        self.x_flhx_command, self.y_flhx_command = [], []
        self.x_frhx_command, self.y_frhx_command = [], []
        self.x_rlhx_command, self.y_rlhx_command = [], []
        self.x_rrhx_command, self.y_rrhx_command = [], []
        self.x_flhy_command, self.y_flhy_command = [], []
        self.x_frhy_command, self.y_frhy_command = [], []
        self.x_rlhy_command, self.y_rlhy_command = [], []
        self.x_rrhy_command, self.y_rrhy_command = [], []
        self.x_flk_command, self.y_flk_command = [], []
        self.x_frk_command, self.y_frk_command = [], []
        self.x_rlk_command, self.y_rlk_command = [], []
        self.x_rrk_command, self.y_rrk_command = [], []

        self.x_flhx_state, self.y_flhx_state = [], []
        self.x_frhx_state, self.y_frhx_state = [], []
        self.x_rlhx_state, self.y_rlhx_state = [], []
        self.x_rrhx_state, self.y_rrhx_state = [], []
        self.x_flhy_state, self.y_flhy_state = [], []
        self.x_frhy_state, self.y_frhy_state = [], []
        self.x_rlhy_state, self.y_rlhy_state = [], []
        self.x_rrhy_state, self.y_rrhy_state = [], []
        self.x_flk_state, self.y_flk_state = [], []
        self.x_frk_state, self.y_frk_state = [], []
        self.x_rlk_state, self.y_rlk_state = [], []
        self.x_rrk_state, self.y_rrk_state = [], []

        self.ax01.set_xlim(0, 10000)
        self.ax01.set_ylim(-3, 3)
        self.ax02.set_xlim(0, 10000)
        self.ax02.set_ylim(-3, 3)
        self.ax03.set_xlim(0, 10000)
        self.ax03.set_ylim(-3, 3)
        self.ax04.set_xlim(0, 10000)
        self.ax04.set_ylim(-3, 3)
        self.ax05.set_xlim(0, 10000)
        self.ax05.set_ylim(-3, 3)
        self.ax06.set_xlim(0, 10000)
        self.ax06.set_ylim(-3, 3)
        self.ax07.set_xlim(0, 10000)
        self.ax07.set_ylim(-3, 3)
        self.ax08.set_xlim(0, 10000)
        self.ax08.set_ylim(-3, 3)
        self.ax09.set_xlim(0, 10000)
        self.ax09.set_ylim(-3, 3)
        self.ax10.set_xlim(0, 10000)
        self.ax10.set_ylim(-3, 3)
        self.ax11.set_xlim(0, 10000)
        self.ax11.set_ylim(-3, 3)
        self.ax12.set_xlim(0, 10000)
        self.ax12.set_ylim(-3, 3)

    def plot_init(self):
        self.ax01.set_xlim(0, 10000)
        self.ax01.set_ylim(-3, 3)
        self.ax02.set_xlim(0, 10000)
        self.ax02.set_ylim(-3, 3)
        self.ax03.set_xlim(0, 10000)
        self.ax03.set_ylim(-3, 3)
        self.ax04.set_xlim(0, 10000)
        self.ax04.set_ylim(-3, 3)
        self.ax05.set_xlim(0, 10000)
        self.ax05.set_ylim(-3, 3)
        self.ax06.set_xlim(0, 10000)
        self.ax06.set_ylim(-3, 3)
        self.ax07.set_xlim(0, 10000)
        self.ax07.set_ylim(-3, 3)
        self.ax08.set_xlim(0, 10000)
        self.ax08.set_ylim(-3, 3)
        self.ax09.set_xlim(0, 10000)
        self.ax09.set_ylim(-3, 3)
        self.ax10.set_xlim(0, 10000)
        self.ax10.set_ylim(-3, 3)
        self.ax11.set_xlim(0, 10000)
        self.ax11.set_ylim(-3, 3)
        self.ax12.set_xlim(0, 10000)
        self.ax12.set_ylim(-3, 3)

    def callback_front_left_hip_x(self, data):
        y = data.process_value
        self.y_flhx_state.append(y)
        x_index = len(self.x_flhx_state)
        self.x_flhx_state.append(x_index + 1)

    def callback_front_left_hip_y(self, data):
        y = data.process_value
        self.y_flhy_state.append(y)
        x_index = len(self.x_flhy_state)
        self.x_flhy_state.append(x_index + 1)

    def callback_front_left_knee(self, data):
        y = data.process_value
        self.y_flk_state.append(y)
        x_index = len(self.x_flk_state)
        self.x_flk_state.append(x_index + 1)

    def callback_front_right_hip_x(self, data):
        y = data.process_value
        self.y_frhx_state.append(y)
        x_index = len(self.x_frhx_state)
        self.x_frhx_state.append(x_index + 1)

    def callback_front_right_hip_y(self, data):
        y = data.process_value
        self.y_frhy_state.append(y)
        x_index = len(self.x_frhy_state)
        if isinstance(x_index, int):
           print("True")
        else:
            print("False")
            time.sleep(100)

        self.x_frhy_state.append(x_index + 1.0)
        # print("x_index=", x_index)
        # print("y=", y)

    def callback_front_right_knee(self, data):
        y = data.process_value
        self.y_frk_state.append(y)
        x_index = len(self.x_frk_state)
        self.x_frk_state.append(x_index + 1)

    def callback_rear_left_hip_x(self, data):
        y = data.process_value
        self.y_rlhx_state.append(y)
        x_index = len(self.x_rlhx_state)
        self.x_rlhx_state.append(x_index + 1)

    def callback_rear_left_hip_y(self, data):
        y = data.process_value
        self.y_rlhy_state.append(y)
        x_index = len(self.x_rlhy_state)
        self.x_rlhy_state.append(x_index + 1)

    def callback_rear_left_knee(self, data):
        y = data.process_value
        self.y_rlk_state.append(y)
        x_index = len(self.x_rlk_state)
        self.x_rlk_state.append(x_index + 1)

    def callback_rear_right_hip_x(self, data):
        y = data.process_value
        self.y_rrhy_state.append(y)
        x_index = len(self.x_rrhy_state)
        self.x_rrhy_state.append(x_index + 1)

    def callback_rear_right_hip_y(self, data):
        y = data.process_value
        self.y_frhy_state.append(y)
        x_index = len(self.x_rrhy_state)
        self.x_rrhy_state.append(x_index + 1)

    def callback_rear_right_knee(self, data):
        y = data.process_value
        self.y_rrk_state.append(y)
        x_index = len(self.x_rrk_state)
        self.x_rrk_state.append(x_index + 1)
        # print("x_index=", x_index)
        # print("y=", y)

    def update_plot(self, frame):
        # self.p011.set_data(self.x_flhx_state, self.y_flhx_state)
        # self.p021.set_data(self.x_flhy_state, self.y_flhy_state)
        # self.p031.set_data(self.x_flk_state, self.y_flk_state)
        # self.p041.set_data(self.x_frhx_state, self.y_frhx_state)
        self.p051.set_data(self.x_frhy_state, self.y_frhy_state)
        # self.p061.set_data(self.x_frk_state, self.y_frk_state)
        # self.p071.set_data(self.x_rlhx_state, self.y_rlhx_state)
        # self.p081.set_data(self.x_rlhy_state, self.y_rlhy_state)
        # self.p091.set_data(self.x_rlk_state, self.y_rlk_state)
        # self.p101.set_data(self.x_rrhx_state, self.y_rrhx_state)
        # self.p111.set_data(self.x_rrhy_state, self.y_rrhy_state)
        # self.p121.set_data(self.x_rrk_state, self.y_rrk_state)




rospy.init_node('spot_plot_visual_node')
vis = Visualiser()
rospy.Subscriber("/spot/joint_front_left_hip_x_controller/state", JointControllerState, vis.callback_front_left_hip_x)
rospy.Subscriber("/spot/joint_front_left_hip_y_controller/state", JointControllerState, vis.callback_front_left_hip_y)
rospy.Subscriber("/spot/joint_front_left_knee_controller/state", JointControllerState, vis.callback_front_left_knee)

rospy.Subscriber("/spot/joint_front_right_hip_x_controller/state", JointControllerState, vis.callback_front_right_hip_x)
rospy.Subscriber("/spot/joint_front_right_hip_y_controller/state", JointControllerState, vis.callback_front_right_hip_y)
rospy.Subscriber("/spot/joint_front_right_knee_controller/state", JointControllerState, vis.callback_front_right_knee)

rospy.Subscriber("/spot/joint_rear_left_hip_x_controller/state", JointControllerState, vis.callback_rear_left_hip_x)
rospy.Subscriber("/spot/joint_rear_left_hip_y_controller/state", JointControllerState, vis.callback_rear_left_hip_y)
rospy.Subscriber("/spot/joint_rear_left_knee_controller/state", JointControllerState, vis.callback_rear_left_knee)

rospy.Subscriber("/spot/joint_rear_right_hip_x_controller/state", JointControllerState, vis.callback_rear_right_hip_x)
rospy.Subscriber("/spot/joint_rear_right_hip_y_controller/state", JointControllerState, vis.callback_rear_right_hip_y)
rospy.Subscriber("/spot/joint_rear_right_knee_controller/state", JointControllerState, vis.callback_rear_right_knee)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, blit=False, frames=10, interval=10000, repeat=False)
plt.pause(100.5)
plt.show(block=True)
