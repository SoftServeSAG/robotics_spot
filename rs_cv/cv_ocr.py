import pytesseract
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from pytesseract import Output
import re


def perform_awesome_detection(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thres = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    custom_config = r'-l eng --oem 3 --psm 6' 
    d = pytesseract.image_to_data(thres, config=custom_config, output_type=Output.DICT, lang='eng')
    n_boxes = len(d['level'])

    temp_num = '21'
    trigger_words = np.array(['Cond', 'cool', 'cooling', 'Cool', 'Cooling', 'on', 'cond', 'mode', 'is', 'Cond.', 'Temp', 'Temp:', temp_num])
    known_words_mask = [word in trigger_words or '21' in word for word in d['text']]

    cond_is_on = any(known_words_mask)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    thickness = 2

    if cond_is_on:
        img = cv2.putText(img, 'Cond. is ON', (50, 50), font, fontScale, (0, 0, 255), thickness, cv2.LINE_AA)
    else:
        img = cv2.putText(img, 'Cond. is OFF', (50, 50), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA)

    for i in range(n_boxes):
        if not known_words_mask[i]:
            continue
        (x, y, w, h) = (d['left'][i], d['top'][i], d['width'][i], d['height'][i])
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        if temp_num in d['text'][i]:
            img = cv2.putText(img, 'T: ' + temp_num, (x - 20, y + 35), font, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.imshow('img', img)
    cv2.waitKey(200)



def image_callback(img_msg):
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
