#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(100)  # 100hz

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

def talker():

    while not rospy.is_shutdown():
        flhx = -2
        frhx = 2
        rlhx = -2
        rrhx = 2
        rospy.loginfo(flhx)
        # Hips in x-direction
        #pub_front_left_hip_x.publish(flhx)
        #pub_front_right_hip_x.publish(frhx)
        #pub_rear_left_hip_x.publish(rlhx)
        #pub_rear_right_hip_x.publish(rrhx)

        # Knee
        pub_front_left_knee.publish(-2.6)
        pub_front_right_knee.publish(-2.6)
        pub_rear_left_knee.publish(-2.6)
        pub_rear_right_knee.publish(-2.6)



        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
