#!/usr/bin/env python
import rospy
from control_msgs.msg import JointControllerState

def callback_front_left_hip_x(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard front_left_hip_x%s", data.process_value)


def callback_front_left_hip_y(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard front_left_hip_y%s", data.process_value)


def callback_front_left_knee(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard front_left_knee%s", data.process_value)


def callback_front_right_hip_x(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_front_right_hip_y(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_front_right_knee(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_rear_left_hip_x(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_rear_left_hip_y(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_rear_left_knee(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_rear_right_hip_x(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_rear_right_hip_y(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def callback_rear_right_knee(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.process_value)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    spot_name = str(input("Tell me spot name: "))
    rospy.init_node(spot_name + '_listener', anonymous=True)
    rospy.Subscriber('/' + spot_name + '/joint_front_left_hip_x_controller/state', JointControllerState, callback_front_left_hip_x)
    rospy.Subscriber('/' + spot_name + '/joint_front_left_hip_y_controller/state', JointControllerState, callback_front_left_hip_y)
    rospy.Subscriber('/' + spot_name + '/joint_front_left_knee_controller/state', JointControllerState, callback_front_left_knee)

    rospy.Subscriber('/' + spot_name + '/joint_front_right_hip_x_controller/state', JointControllerState, callback_front_right_hip_x)
    rospy.Subscriber('/' + spot_name + '/joint_front_right_hip_y_controller/state', JointControllerState, callback_front_right_hip_y)
    rospy.Subscriber('/' + spot_name + '/joint_front_right_knee_controller/state', JointControllerState, callback_front_right_knee)

    rospy.Subscriber('/' + spot_name + '/joint_rear_left_hip_x_controller/state', JointControllerState, callback_rear_left_hip_x)
    rospy.Subscriber('/' + spot_name + '/joint_rear_left_hip_y_controller/state', JointControllerState, callback_rear_left_hip_y)
    rospy.Subscriber('/' + spot_name + '/joint_rear_left_knee_controller/state', JointControllerState, callback_rear_left_knee)

    rospy.Subscriber('/' + spot_name + '/joint_rear_right_hip_x_controller/state', JointControllerState, callback_rear_right_hip_x)
    rospy.Subscriber('/' + spot_name + '/joint_rear_right_hip_y_controller/state', JointControllerState, callback_rear_right_hip_y)
    rospy.Subscriber('/' + spot_name + '/joint_rear_right_knee_controller/state', JointControllerState, callback_rear_right_knee)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()