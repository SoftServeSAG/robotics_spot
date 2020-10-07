#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(1000)  # 100hz

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



        

if __name__ == '__main__':
    motors_target_pos = [[0.20, 1.0, -1.69],  # Front left leg
                         [-0.20, 1.0, -1.69],  # Front right leg
                         [0.20, 1.0, -1.69],  # Rear left leg
                         [-0.20, 1.0, -1.69]]  # Rear right leg
    try:
        while not rospy.is_shutdown():
            talker()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
