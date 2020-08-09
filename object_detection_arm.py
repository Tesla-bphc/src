#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt
from grid_arm_full_24.msg import location
from detect import detect
boxes=[]
done = 1
bridge = CvBridge()
pub = rospy.Publisher('location', location, queue_size=1)
def callback(data):
    global done
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    #frame = cv2.flip(frame,1)
    boxes = detect(frame)
    box = location()
    if boxes != [] and done:
        box.x = boxes[0]
        box.y = boxes[1]
        box.width = boxes[2]
        box.height = boxes[3]
        rospy.loginfo(box)
        pub.publish(box)
        xp = boxes[0]
        yp = boxes[1]
        done = 0
    else:
        box.x = 0
        box.y = 0
        box.width = 0
        box.height = 0
        pub.publish(box)
    boxes=[]
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("rgb/camera/image_raw", Image, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
