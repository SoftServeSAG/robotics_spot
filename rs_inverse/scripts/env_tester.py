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

from simple_pid import PID

import time

from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('inverse')
rate = rospy.Rate(100)  # 100hz

robot_name = rospy.get_param("/name_robot_1")

# -------------------------------------- Publishers -------------------------------------------------------
# ----Front ----
# left front hip_x
pub_front_left_hip_x = rospy.Publisher('/' + robot_name + '/joint_front_left_hip_x_controller/command',
                                       Float64, queue_size=10)

# left front hip_y
pub_front_left_hip_y = rospy.Publisher('/' + robot_name + '/joint_front_left_hip_y_controller/command',
                                       Float64, queue_size=10)

# left front knee
pub_front_left_knee = rospy.Publisher('/' + robot_name + '/joint_front_left_knee_controller/command',
                                      Float64, queue_size=10)

# right front hip_x
pub_front_right_hip_x = rospy.Publisher('/' + robot_name + '/joint_front_right_hip_x_controller/command',
                                        Float64, queue_size=10)

# right front hip_y
pub_front_right_hip_y = rospy.Publisher('/' + robot_name + '/joint_front_right_hip_y_controller/command',
                                        Float64, queue_size=10)

# right front knee
pub_front_right_knee = rospy.Publisher('/' + robot_name + '/joint_front_right_knee_controller/command',
                                       Float64, queue_size=10)

# ---- Rear ----

# left rear hip_x
pub_rear_left_hip_x = rospy.Publisher('/' + robot_name + '/joint_rear_left_hip_x_controller/command',
                                      Float64, queue_size=10)

# left rear hip_y
pub_rear_left_hip_y = rospy.Publisher('/' + robot_name + '/joint_rear_left_hip_y_controller/command',
                                      Float64, queue_size=10)

# left rear knee
pub_rear_left_knee = rospy.Publisher('/' + robot_name + '/joint_rear_left_knee_controller/command',
                                     Float64, queue_size=10)

# right rear hip_x
pub_rear_right_hip_x = rospy.Publisher('/' + robot_name + '/joint_rear_right_hip_x_controller/command',
                                       Float64, queue_size=10)

# right rear hip_y
pub_rear_right_hip_y = rospy.Publisher('/' + robot_name + '/joint_rear_right_hip_y_controller/command',
                                       Float64, queue_size=10)

# right rear knee
pub_rear_right_knee = rospy.Publisher('/' + robot_name + '/joint_rear_right_knee_controller/command',
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
YawControl = 0.0


# ------------- Subscribers ------------


def callback_gait(data):
    global xd, yd, zd, rolld, pitchd, yawd, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, \
        PenetrationDepth, SwingPeriod, YawControl
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
    YawControl = data.YawControl


global x_inst, y_inst, z_inst, roll_inst, pitch_inst, yaw_inst, vx_inst, vy_inst, vz_inst, search_index
x_inst = 0
y_inst = 0
z_inst = 0
roll_inst = 0
pitch_inst = 0
yaw_inst = 0
vx_inst = 0
vy_inst = 0
vz_inst = 0
wz_inst = 0
search_index = -1


def callback_model(data):
    global x_inst, y_inst, z_inst, roll_inst, pitch_inst, yaw_inst, vx_inst, vy_inst, vz_inst, wz_inst, search_index
    if search_index == -1:
        search_index = data.name.index('spot_mini')
    x_inst = data.pose[search_index].position.x
    y_inst = data.pose[search_index].position.y
    z_inst = data.pose[search_index].position.z
    orientation_q = data.pose[search_index].orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll_inst, pitch_inst, yaw_inst) = euler_from_quaternion(orientation_list)


global front_left_lower_leg_contact, front_right_lower_leg_contact, rear_left_lower_leg_contact, rear_right_lower_leg_contact
global chattering_front_left_lower_leg_contact, chattering_front_right_lower_leg_contact, \
       chattering_rear_left_lower_leg_contact, chattering_rear_right_lower_leg_contact, lim_chattering
front_left_lower_leg_contact = 1
front_right_lower_leg_contact = 1
rear_left_lower_leg_contact = 1
rear_right_lower_leg_contact = 1
chattering_front_left_lower_leg_contact = 0
chattering_front_right_lower_leg_contact = 0
chattering_rear_left_lower_leg_contact = 0
chattering_rear_right_lower_leg_contact = 0
lim_chattering = 4


def callback_front_left_lower_leg_contact(data):
    global front_left_lower_leg_contact, chattering_front_left_lower_leg_contact, lim_chattering
    # rospy.loginfo_throttle(0.0005, "chattering = " + str(chattering_front_left_lower_leg_contact))
    if len(data.states) == 0:
        chattering_front_left_lower_leg_contact += 1
        if chattering_front_left_lower_leg_contact > lim_chattering:
            front_left_lower_leg_contact = 0
    else:
        front_left_lower_leg_contact = 1
        chattering_front_left_lower_leg_contact = 0
        # rospy.loginfo_throttle(0.0005, "len = " + str(data.states[0].contact_normals[0].z))


def callback_front_right_lower_leg_contact(data):
    global front_right_lower_leg_contact, chattering_front_right_lower_leg_contact, lim_chattering
    if len(data.states) == 0:
        chattering_front_right_lower_leg_contact += 1
        if chattering_front_right_lower_leg_contact > lim_chattering:
            front_right_lower_leg_contact = 0
    else:
        front_right_lower_leg_contact = 1
        chattering_front_right_lower_leg_contact = 0


def callback_rear_left_lower_leg_contact(data):
    global rear_left_lower_leg_contact, chattering_rear_left_lower_leg_contact, lim_chattering
    if len(data.states) == 0:
        chattering_rear_left_lower_leg_contact += 1
        if chattering_rear_left_lower_leg_contact > lim_chattering:
            rear_left_lower_leg_contact = 0
    else:
        rear_left_lower_leg_contact = 1
        chattering_rear_left_lower_leg_contact = 0


def callback_rear_right_lower_leg_contact(data):
    global rear_right_lower_leg_contact, chattering_rear_right_lower_leg_contact, lim_chattering
    if len(data.states) == 0:
        chattering_rear_right_lower_leg_contact += 1
        if chattering_rear_right_lower_leg_contact > lim_chattering:
            rear_right_lower_leg_contact = 0
    else:
        rear_right_lower_leg_contact = 1
        chattering_rear_right_lower_leg_contact = 0


def listener():
    rospy.Subscriber("/spot/inverse_gait_input", GaitInput, callback_gait)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback_model)
    rospy.Subscriber("/front_left_lower_leg_contact", ContactsState, callback_front_left_lower_leg_contact)
    rospy.Subscriber("/front_right_lower_leg_contact", ContactsState, callback_front_right_lower_leg_contact)
    rospy.Subscriber("/rear_left_lower_leg_contact", ContactsState, callback_rear_left_lower_leg_contact)
    rospy.Subscriber("/rear_right_lower_leg_contact", ContactsState, callback_rear_right_lower_leg_contact)


def main():
    """ The main() function. """

    print("STARTING SPOT TEST ENV")
    time_step = 0.01
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
    time.sleep(1.1)

    rospy.loginfo_once("STARTED SPOT TEST ENV")
    t = 0
    global xd, yd, zd, rolld, pitchd, yawd, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, \
        PenetrationDepth, SwingPeriod, YawControl
    global x_inst, y_inst, z_inst, roll_inst, pitch_inst, yaw_inst, vx_inst, vy_inst, vz_inst, wz_inst
    global front_left_lower_leg_contact, front_right_lower_leg_contact, rear_left_lower_leg_contact, rear_right_lower_leg_contact

    listener()


    pid = PID(1.1, 0.01, 0.000005)
    pid.sample_time = 0.01
    pid.output_limits = (-10, 10)
    # pid.proportional_on_measurement = True

    while t < (int(max_timesteps)):
        start_time = time.time()

        pos = np.array([xd, yd, zd])
        orn = np.array([rolld, pitchd, yawd])

        # Update Swing Period
        bzg.Tswing = SwingPeriod

        # rospy.loginfo_throttle(0.005, "contact front left = " + str(front_left_lower_leg_contact))
        # rospy.loginfo_throttle(0.005, "contact front left = " + str(front_right_lower_leg_contact))
        # rospy.loginfo_throttle(0.005, "contact rear left = " + str(rear_left_lower_leg_contact))
        # rospy.loginfo_throttle(0.005, "contact rear right = " + str(rear_right_lower_leg_contact))

        yaw_target = YawControl
        pid.setpoint = yaw_target
        rospy.loginfo_once("y_target_init=" + str(yaw_target))

        residual = yaw_target - yaw_inst
        if abs(residual) < 0.1:
            p_yaw = 1.0
        elif 0.1 <= abs(residual) < 1.1:
            p_yaw = 3.5
        else:
            p_yaw = 2.0

        rospy.loginfo_throttle(1, "residual=" + str(yaw_target - yaw_inst))

        pid.Kp = p_yaw
        YawRate_d = pid(yaw_inst)

        contacts = [front_left_lower_leg_contact, front_right_lower_leg_contact, rear_left_lower_leg_contact,
                    rear_right_lower_leg_contact]

        # Get Desired Foot Poses
        T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate_d,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth,
                                      contacts)
        joint_angles = spot.IK(orn, pos, T_bf)


        t += 1
        # print(joint_angles)
        talker(joint_angles)
        # your code
        elapsed_time = time.time() - start_time
        pid.sample_time = elapsed_time


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
