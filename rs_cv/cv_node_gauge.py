import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math


def remap(OldValue, OldMin, OldMax, NewMin, NewMax):
    return (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def perform_awesome_localization(img_prev, img):
    output = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(cv2.medianBlur(hsv[..., 0], 15), 50, 255)
    arrow_contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]

    _, triangle = cv2.minEnclosingTriangle(arrow_contours[0])
    rectangle = cv2.minAreaRect(arrow_contours[0])

    box = cv2.boxPoints(rectangle)
    box = np.int0(box)
    cv2.drawContours(output, [box], 0, (0, 0, 255), 2)

    blurred = cv2.inRange(cv2.medianBlur(gray, 15), 0, 10)

    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 2.5, 100)

    if circles is not None:
        if len(circles > 0):
            circles = np.round(circles[0, :]).astype("int")
            circle_x, circle_y, circle_r = circles[0]
            cv2.circle(output, (circle_x, circle_y), circle_r, (0, 255, 0), 4)
            cv2.rectangle(output, (circle_x - 5, circle_y - 5), (circle_x + 5, circle_y + 5), (0, 128, 255), -1)

    max_dist = 0
    furthest_point = None
    for i in triangle:
        tri_x, tri_y = np.round(i[0]).astype("int")
        curr_dist = (circle_x - tri_x) ** 2 + (circle_y - tri_y) ** 2
        if curr_dist > max_dist:
            max_dist = curr_dist
            furthest_point = (tri_x, tri_y)

    cv2.circle(output, furthest_point, 2, (255, 255, 0), 4)

    x1 = circle_x - furthest_point[0]
    y1 = circle_y - furthest_point[1]
    x2 = 0
    y2 = -1
    dot = x1 * x2 + y1 * y2  # dot product
    det = x1 * y2 - y1 * x2  # determinant
    angle = math.atan2(det, dot) * 360 / (2 * math.pi)
    if angle < 0:
        angle += 360
    angle = 360 - angle

    value = round(clamp(36 - remap(angle, 0, 360, 36, 0), 7, 29) - 7, 1)

    output = cv2.putText(output, 'GAUGE : ' + str(value), (circle_x - circle_r, circle_y - circle_r - 20),
                         cv2.FONT_HERSHEY_SIMPLEX,
                         0.8, (0, 255, 0), 2, cv2.LINE_AA)

    res = output
    return res


global prev_img
prev_img = None


def image_callback(img_msg):
    global prev_img
    print("Received image")
    try:
        img = CvBridge().imgmsg_to_cv2(img_msg)
        # cv2.imshow('show', img)
        cv2.waitKey(1)
        if prev_img is not None:
            res = perform_awesome_localization(prev_img, img)
            cv2.imshow("localization demo", res)
        prev_img = img
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
