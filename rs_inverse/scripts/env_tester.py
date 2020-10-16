#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import copy
import rospy
from std_msgs.msg import Float64
from rs_msgs.msg import GaitInput

from SpotKinematics import SpotModel
from LieAlgebra import RPY

from Bezier import BezierGait

# TESTING
from SpotOL import BezierStepper

import time
import os

rospy.init_node('inverse', anonymous=True)
rate = rospy.Rate(100)  # 100hz

# -------------------------------------- Publishers -------------------------------------------------------
# ----Front ----
# left front hip_x
pub_front_left_hip_x = rospy.Publisher('/spot/joint_front_left_hip_x_controller/command',
                                       Float64, queue_size=10)

# left front hip_y
pub_front_left_hip_y = rospy.Publisher('/spot/joint_front_left_hip_y_controller/command',
                                       Float64, queue_size=10)

# left front knee
pub_front_left_knee = rospy.Publisher('/spot/joint_front_left_knee_controller/command',
                                      Float64, queue_size=10)


# right front hip_x
pub_front_right_hip_x = rospy.Publisher('/spot/joint_front_right_hip_x_controller/command',
                                        Float64, queue_size=10)

# right front hip_y
pub_front_right_hip_y = rospy.Publisher('/spot/joint_front_right_hip_y_controller/command',
                                        Float64, queue_size=10)

# right front knee
pub_front_right_knee = rospy.Publisher('/spot/joint_front_right_knee_controller/command',
                                       Float64, queue_size=10)

# ---- Rear ----

# left rear hip_x
pub_rear_left_hip_x = rospy.Publisher('/spot/joint_rear_left_hip_x_controller/command',
                                      Float64, queue_size=10)

# left rear hip_y
pub_rear_left_hip_y = rospy.Publisher('/spot/joint_rear_left_hip_y_controller/command',
                                      Float64, queue_size=10)

# left rear knee
pub_rear_left_knee = rospy.Publisher('/spot/joint_rear_left_knee_controller/command',
                                     Float64, queue_size=10)

# right rear hip_x
pub_rear_right_hip_x = rospy.Publisher('/spot/joint_rear_right_hip_x_controller/command',
                                       Float64, queue_size=10)

# right rear hip_y
pub_rear_right_hip_y = rospy.Publisher('/spot/joint_rear_right_hip_y_controller/command',
                                       Float64, queue_size=10)

# right rear knee
pub_rear_right_knee = rospy.Publisher('/spot/joint_rear_right_knee_controller/command',
                                      Float64, queue_size=10)


def talker(motors_target_pos):
    # Hips in x-direction
    pub_front_left_hip_x.publish(motors_target_pos[0][0])
    pub_front_right_hip_x.publish(motors_target_pos[1][0])
    pub_rear_left_hip_x.publish(motors_target_pos[2][0])
    pub_rear_right_hip_x.publish(motors_target_pos[3][0])

    # Hips in y-direction
    pub_front_left_hip_y.publish(motors_target_pos[0][1])
    pub_front_right_hip_y.publish(motors_target_pos[1][1])
    pub_rear_left_hip_y.publish(motors_target_pos[2][1])
    pub_rear_right_hip_y.publish(motors_target_pos[3][1])

    # Knee
    pub_front_left_knee.publish(motors_target_pos[0][2])
    pub_front_right_knee.publish(motors_target_pos[1][2])
    pub_rear_left_knee.publish(motors_target_pos[2][2])
    pub_rear_right_knee.publish(motors_target_pos[3][2])


# ------------------ Standard pose
sit_down = [[0.20, 1.0, -2.49],  # Front left leg
            [-0.20, 1.0, -2.49],  # Front right leg
            [0.20, 1.0, -2.49],  # Rear left leg
            [-0.20, 1.0, -2.49]]  # Rear right leg

stand_up = [[0.20, 0.7, -1.39],  # Front left leg
            [-0.20, 0.7, -1.39],  # Front right leg
            [0.20, 0.7, -1.39],  # Rear left leg
            [-0.20, 0.7, -1.39]]  # Rear right leg


# ------------------ Inputs for Bezier Gait control ----------------
global xd, yd, zd, rolld, pitchd, yawd, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, \
    PenetrationDepth, SwingPeriod
xd = 0.0
yd = 0.0
zd = 0.0
rolld = 0.0
pitchd = 0.0
yawd = 0.0
StepLength = 0.00
LateralFraction = 0.0
YawRate = 0.0
StepVelocity = 0.00
ClearanceHeight = 0.0
PenetrationDepth = 0.0
SwingPeriod = 0.00

# ------------- Subscribers ------------


def callback_gait(data):
    global xd, yd, zd, rolld, pitchd, yawd, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, \
        PenetrationDepth, SwingPeriod
    xd = data.x
    yd = data.y
    zd = data.z
    rolld = data.roll
    pitchd = data.pitch
    yawd = data.yaw
    StepLength = data.StepLength
    LateralFraction = data.LateralFraction
    YawRate = data.YawRate
    StepVelocity = data.StepVelocity
    ClearanceHeight = data.ClearanceHeight
    PenetrationDepth = data.PenetrationDepth
    SwingPeriod = data.SwingPeriod


def listener():
    rospy.Subscriber("/spot/inverse_gait_input", GaitInput, callback_gait)


def main():
    """ The main() function. """

    print("STARTING SPOT TEST ENV")
    time_step = 0.001
    max_timesteps = 4e6

    spot = SpotModel()
    T_bf0 = spot.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)

    bzg = BezierGait(dt=time_step)

    # sit down
    time.sleep(2.1)
    talker(sit_down)
    rate.sleep()
    # stand up
    time.sleep(2.1)
    talker(stand_up)
    rate.sleep()
    time.sleep(3.1)

    print("STARTED SPOT TEST ENV")
    t = 0
    global xd, yd, zd, rolld, pitchd, yawd, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, \
        PenetrationDepth, SwingPeriod

    while t < (int(max_timesteps)):
        listener()

        pos = np.array([xd, yd, zd])
        orn = np.array([rolld, pitchd, yawd])

        # Update Swing Period
        bzg.Tswing = SwingPeriod

        # print("YAW RATE: {}".format(YawRate))

        # contacts = state[-4:]

        contacts = [0, 0, 0, 0]  # Should get from gazebo

        # Get Desired Foot Poses
        T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth,
                                      contacts)
        joint_angles = spot.IK(orn, pos, T_bf)


        # for i, (key, Tbf_in) in enumerate(T_bf.items()):
        #     print("{}: \t Angle: {}".format(key, np.degrees(joint_angles[i])))
        # print("-------------------------")

        t += 1
        # print(joint_angles)
        talker(joint_angles)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
