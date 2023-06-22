import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)


def perform_awesome_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray, dictionary)

    if len(res[0])>0:
        print("Detected a marker")
        cv2.aruco.drawDetectedMarkers(gray, res[0], res[1])

    cv2.imshow('frame', gray)
    cv2.waitKey(1)


def image_callback(img_msg):
    print("Received image")
    try:
        img = CvBridge().imgmsg_to_cv2(img_msg)
        perform_awesome_detection(img)
    except CvBridgeError:
        rospy.logerr("Failed to convert ros img to opencv")


def create_sub():
    return rospy.Subscriber("/spot1/spot/camera1/image_raw", Image, image_callback)


def shotdown_hook():
    print("CV node shotdown")


if __name__ == '__main__':
    rospy.init_node("cv_node", log_level=rospy.INFO)
    sub = create_sub()
    while not rospy.is_shutdown():
        rospy.spin()
