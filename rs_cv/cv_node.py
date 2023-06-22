import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def perform_awesome_localization(img_prev, img):
    feature_extractor = cv2.ORB_create()
    img_prev = cv2.rotate(img_prev, cv2.ROTATE_90_CLOCKWISE)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    kp1, des1 = feature_extractor.detectAndCompute(img_prev, None)
    kp2, des2 = feature_extractor.detectAndCompute(img, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(des1, des2, k=2)
    good = []
    for m, n in matches:
        if m.distance < 0.5 * n.distance:
            good.append(m)
    if len(good) == 0:
        return None
    res = cv2.drawMatches(img_prev, kp1, img, kp2, good, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    res = cv2.rotate(res, cv2.ROTATE_90_COUNTERCLOCKWISE)
    res = cv2.resize(res, (1000, 500))
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
