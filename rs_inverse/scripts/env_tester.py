#!/usr/bin/env python

import numpy as np
import copy
import rospy
from std_msgs.msg import Float64
from rs_msgs.msg import GaitInput
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ContactsState, ContactState

from SpotKinematics import SpotModel
from LieAlgebra import RPY

from Bezier import BezierGait

from StairsGait import StairsGait

import time

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# spot_name = str(input("Tell me spot name: "))

spot_name = "spot1"

class SpotMidlware:
    def __init__(self, spot_name, time_step):
        self.spot_name = spot_name
        rospy.init_node(self.spot_name + '_inverse')
        self.rate = rospy.Rate(100)  # 100hz
        self.time_step = time_step

        self.spot = SpotModel()
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=self.time_step)

        self.stairs_gait = StairsGait(h_raise=0.168125, w_run=0.25, beta=0.8, delta_h=0.3, t_swing=0.85,
                                      T_bf0=self.T_bf0, time_step=self.time_step)
        self.init_stairs = False

        # ------------------ Inputs for Bezier Gait control ----------------
        self.xd = 0.0
        self.yd = 0.0
        self.zd = 0.0
        self.rolld = 0.0
        self.pitchd = 0.0
        self.yawd = 0.0
        self.StepLength = 0.00
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        self.StepVelocity = 0.00
        self.ClearanceHeight = 0.0
        self.PenetrationDepth = 0.0
        self.SwingPeriod = 0.00
        self.YawControl = 0.0

        # ------------------ Spot states ----------------
        self.x_inst = 0.
        self.y_inst = 0.
        self.z_inst = 0.
        self.roll_inst = 0.
        self.pitch_inst = 0.
        self.yaw_inst = 0.
        self.search_index = -1

        # ------------------ Outputs of Contact sensors ----------------
        self.front_left_lower_leg_contact = 1
        self.front_right_lower_leg_contact = 1
        self.rear_left_lower_leg_contact = 1
        self.rear_right_lower_leg_contact = 1
        self.chattering_front_left_lower_leg_contact = 0
        self.chattering_front_right_lower_leg_contact = 0
        self.chattering_rear_left_lower_leg_contact = 0
        self.chattering_rear_right_lower_leg_contact = 0
        self.lim_chattering = 4

        # -------------------------------------- Publishers -------------------------------------------------------
        # ----Front ----
        # left front hip_x
        self.pub_front_left_hip_x = rospy.Publisher('/' + self.spot_name + '/joint_front_left_hip_x_controller/command',
                                                    Float64, queue_size=1)

        # left front hip_y
        self.pub_front_left_hip_y = rospy.Publisher('/' + self.spot_name + '/joint_front_left_hip_y_controller/command',
                                                    Float64, queue_size=1)

        # left front knee
        self.pub_front_left_knee = rospy.Publisher('/' + self.spot_name + '/joint_front_left_knee_controller/command',
                                                   Float64, queue_size=1)

        # right front hip_x
        self.pub_front_right_hip_x = rospy.Publisher(
            '/' + self.spot_name + '/joint_front_right_hip_x_controller/command',
            Float64, queue_size=1)

        # right front hip_y
        self.pub_front_right_hip_y = rospy.Publisher(
            '/' + self.spot_name + '/joint_front_right_hip_y_controller/command',
            Float64, queue_size=1)

        # right front knee
        self.pub_front_right_knee = rospy.Publisher('/' + self.spot_name + '/joint_front_right_knee_controller/command',
                                                    Float64, queue_size=1)

        # ---- Rear ----

        # left rear hip_x
        self.pub_rear_left_hip_x = rospy.Publisher('/' + self.spot_name + '/joint_rear_left_hip_x_controller/command',
                                                   Float64, queue_size=1)

        # left rear hip_y
        self.pub_rear_left_hip_y = rospy.Publisher('/' + self.spot_name + '/joint_rear_left_hip_y_controller/command',
                                                   Float64, queue_size=1)

        # left rear knee
        self.pub_rear_left_knee = rospy.Publisher('/' + self.spot_name + '/joint_rear_left_knee_controller/command',
                                                  Float64, queue_size=1)

        # right rear hip_x
        self.pub_rear_right_hip_x = rospy.Publisher('/' + self.spot_name + '/joint_rear_right_hip_x_controller/command',
                                                    Float64, queue_size=1)

        # right rear hip_y
        self.pub_rear_right_hip_y = rospy.Publisher('/' + self.spot_name + '/joint_rear_right_hip_y_controller/command',
                                                    Float64, queue_size=1)

        # right rear knee
        self.pub_rear_right_knee = rospy.Publisher('/' + self.spot_name + '/joint_rear_right_knee_controller/command',
                                                   Float64, queue_size=1)

    def talker(self, motors_target_pos):
        # Hips in x-direction
        self.pub_front_left_hip_x.publish(motors_target_pos[0][0])
        self.pub_front_right_hip_x.publish(motors_target_pos[1][0])
        self.pub_rear_left_hip_x.publish(motors_target_pos[2][0])
        self.pub_rear_right_hip_x.publish(motors_target_pos[3][0])

        # Hips in y-direction
        self.pub_front_left_hip_y.publish(motors_target_pos[0][1])
        self.pub_front_right_hip_y.publish(motors_target_pos[1][1])
        self.pub_rear_left_hip_y.publish(motors_target_pos[2][1])
        self.pub_rear_right_hip_y.publish(motors_target_pos[3][1])

        # Knee
        self.pub_front_left_knee.publish(motors_target_pos[0][2])
        self.pub_front_right_knee.publish(motors_target_pos[1][2])
        self.pub_rear_left_knee.publish(motors_target_pos[2][2])
        self.pub_rear_right_knee.publish(motors_target_pos[3][2])

    # ------------- Subscribers ------------
    def callback_gait(self, data):
        self.xd = data.x
        self.yd = data.y
        self.zd = data.z
        self.rolld = data.roll
        self.pitchd = data.pitch
        self.yawd = data.yaw
        self.StepLength = data.StepLength
        self.LateralFraction = data.LateralFraction
        self.YawRate = data.YawRate
        self.StepVelocity = data.StepVelocity
        self.ClearanceHeight = data.ClearanceHeight
        self.PenetrationDepth = data.PenetrationDepth
        self.SwingPeriod = data.SwingPeriod
        self.YawControl = data.YawControl

    def callback_model(self, data):
        if self.search_index == -1:
            self.search_index = data.name.index(self.spot_name)
        self.x_inst = data.pose[self.search_index].position.x
        self.y_inst = data.pose[self.search_index].position.y
        self.z_inst = data.pose[self.search_index].position.z
        orientation_q = data.pose[self.search_index].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll_inst, self.pitch_inst, self.yaw_inst) = euler_from_quaternion(orientation_list)

    def callback_front_left_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_front_left_lower_leg_contact += 1
            if self.chattering_front_left_lower_leg_contact > self.lim_chattering:
                self.front_left_lower_leg_contact = 0
        else:
            self.front_left_lower_leg_contact = 1
            self.chattering_front_left_lower_leg_contact = 0

    def callback_front_right_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_front_right_lower_leg_contact += 1
            if self.chattering_front_right_lower_leg_contact > self.lim_chattering:
                self.front_right_lower_leg_contact = 0
        else:
            self.front_right_lower_leg_contact = 1
            self.chattering_front_right_lower_leg_contact = 0

    def callback_rear_left_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_rear_left_lower_leg_contact += 1
            if self.chattering_rear_left_lower_leg_contact > self.lim_chattering:
                self.rear_left_lower_leg_contact = 0
        else:
            self.rear_left_lower_leg_contact = 1
            self.chattering_rear_left_lower_leg_contact = 0

    def callback_rear_right_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_rear_right_lower_leg_contact += 1
            if self.chattering_rear_right_lower_leg_contact > self.lim_chattering:
                self.rear_right_lower_leg_contact = 0
        else:
            self.rear_right_lower_leg_contact = 1
            self.chattering_rear_right_lower_leg_contact = 0

    def listener(self):
        rospy.Subscriber("/" + self.spot_name + "/inverse_gait_input", GaitInput, self.callback_gait)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_model)
        rospy.Subscriber("/" + self.spot_name + "/front_left_lower_leg_contact", ContactsState,
                         self.callback_front_left_lower_leg_contact)
        rospy.Subscriber("/" + self.spot_name + "/front_right_lower_leg_contact", ContactsState,
                         self.callback_front_right_lower_leg_contact)
        rospy.Subscriber("/" + self.spot_name + "/rear_left_lower_leg_contact", ContactsState,
                         self.callback_rear_left_lower_leg_contact)
        rospy.Subscriber("/" + self.spot_name + "/rear_right_lower_leg_contact", ContactsState,
                         self.callback_rear_right_lower_leg_contact)

    def yaw_control(self):
        yaw_target = self.YawControl
        thr = np.pi / 2
        if (yaw_target > thr and self.yaw_inst < -thr) or (self.yaw_inst > thr and yaw_target < -thr):
            residual = (yaw_target - self.yaw_inst) * np.sign(yaw_target - self.yaw_inst) - 2 * np.pi
            yawrate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
        else:
            residual = yaw_target - self.yaw_inst
            yawrate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
        return yawrate_d

    def spot_inverse_control(self):
        pos = np.array([self.xd, self.yd, self.zd])
        orn = np.array([self.rolld, self.pitchd, self.yawd])

        # yaw control
        YawRate_d = self.yaw_control()
        # Update Swing Period
        self.bzg.Tswing = self.SwingPeriod
        contacts = [self.front_left_lower_leg_contact, self.front_right_lower_leg_contact,
                    self.rear_left_lower_leg_contact,
                    self.rear_right_lower_leg_contact]
        # Get Desired Foot Poses
        T_bf = self.bzg.GenerateTrajectory(self.StepLength, self.LateralFraction, YawRate_d,
                                           self.StepVelocity, self.T_bf0, self.T_bf,
                                           self.ClearanceHeight, self.PenetrationDepth,
                                           contacts)
        # rospy.loginfo_throttle(1, "FOOT=" + str(T_bf))
        joint_angles = self.spot.IK(orn, pos, T_bf)
        self.talker(joint_angles)

    def spot_stairs_control(self):
        # if not self.init_stairs:
        #     T_bf = copy.deepcopy(self.T_bf0)
        #     T_bf["FR"][0, 3] += 0.1
        #     joint_angles = self.spot.IK(orn, pos, T_bf)
        #     self.talker(joint_angles)
        #     time.sleep(0.5)
        #     T_bf["BL"][0, 3] -= 0.1
        #     joint_angles = self.spot.IK(orn, pos, T_bf)
        #     self.talker(joint_angles)
        #     time.sleep(0.5)
        #     T_bf["BR"][0, 3] -= 0.05
        #     joint_angles = self.spot.IK(orn, pos, T_bf)
        #     self.talker(joint_angles)
        #     time.sleep(0.5)
        #     T_bf["FL"][0, 3] -= 0.0
        #     joint_angles = self.spot.IK(orn, pos, T_bf)
        #     self.talker(joint_angles)
        #     time.sleep(0.5)
        #     T_bf["BL"][0, 3] += 0.15
        #     joint_angles = self.spot.IK(orn, pos, T_bf)
        #     self.talker(joint_angles)
        #     time.sleep(1.5)
        #     self.init_stairs = True


        contacts = [self.front_left_lower_leg_contact, self.front_right_lower_leg_contact,
                    self.rear_left_lower_leg_contact,
                    self.rear_right_lower_leg_contact]
        # Get Desired Foot Poses
        T_bf = self.stairs_gait.GenerateTrajectory(contacts)
        # rospy.loginfo_throttle(1, "FOOT=" + str(T_bf))
        pos = np.array([self.xd, self.yd, self.zd])
        orn = np.array([self.rolld, self.pitchd, self.yawd])
        joint_angles = self.spot.IK(orn, pos, T_bf)
        self.talker(joint_angles)


# ------------------ Standard pose
sit_down = [[0.20, 1.0, -2.49],  # Front left leg
            [-0.20, 1.0, -2.49],  # Front right leg
            [0.20, 1.0, -2.49],  # Rear left leg
            [-0.20, 1.0, -2.49]]  # Rear right leg

stand_up = [[0.20, 0.7, -1.39],  # Front left leg
            [-0.20, 0.7, -1.39],  # Front right leg
            [0.20, 0.7, -1.39],  # Rear left leg
            [-0.20, 0.7, -1.39]]  # Rear right leg


def main():
    """ The main() function. """

    print("STARTING SPOT TEST ENV")
    time_step = 0.03
    max_timesteps = 4e6

    spot_com = SpotMidlware(spot_name, time_step)

    # sit down
    time.sleep(2.1)
    spot_com.talker(sit_down)
    spot_com.rate.sleep()
    # stand up
    time.sleep(2.1)
    spot_com.talker(stand_up)
    spot_com.rate.sleep()
    time.sleep(1.1)

    rospy.loginfo_once("STARTED SPOT TEST ENV")
    t = 0

    spot_com.listener()

    while t < (int(max_timesteps)):
        start_time = time.time()
        x_start = spot_com.x_inst
        y_start = spot_com.y_inst

        # spot_com.spot_inverse_control()
        spot_com.spot_stairs_control()

        t += 1
        elapsed_time = time.time() - start_time
        vx = (spot_com.x_inst - x_start) / elapsed_time
        vy = (spot_com.y_inst - y_start) / elapsed_time
        vel = np.sqrt(vx ** 2 + vy ** 2)
        # rospy.loginfo_throttle(1, "vx=" + str(vx))
        # rospy.loginfo_throttle(1, "vy=" + str(vy))
        # rospy.loginfo_throttle(1, "vel=" + str(vel))
        # rospy.loginfo_throttle(1, "elapsed_time=" + str(elapsed_time))
        if elapsed_time < spot_com.time_step:
            time.sleep(spot_com.time_step - elapsed_time)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
