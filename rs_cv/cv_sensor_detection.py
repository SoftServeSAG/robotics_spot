import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


def perform_awesome_detection(img):
    gray_im = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    draw_im = img
    
    circles = cv2.HoughCircles(gray_im, cv2.HOUGH_GRADIENT, 1, 40, param1=50, param2=30,minRadius=30,maxRadius=40)
    if circles is None:
        print("Nothing detected")
        return img
    print("Detected num: {}".format(circles))
    circles = np.round(circles[0, :]).astype("int")

    for (x, y, r) in circles:
        cv2.circle(draw_im, (x, y), r, (0, 255, 0), 4)
    return draw_im


def image_callback(img_msg):
    print("Received image")
    try:
        img = CvBridge().imgmsg_to_cv2(img_msg)
        draw_im = perform_awesome_detection(img)
        cv2.imshow("detection", draw_im)
        cv2.waitKey(1)
    except CvBridgeError:
        rospy.logerr("Failed to convert ros img to opencv")


def create_sub():
    return rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)


def shotdown_hook():
    print("CV node shotdown")


if __name__ == '__main__':
    rospy.init_node("cv_node", log_level=rospy.INFO)
    sub = create_sub()
    while not rospy.is_shutdown():
        rospy.spin()
