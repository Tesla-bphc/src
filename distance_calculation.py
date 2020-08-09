#!/usr/bin/env python3
x_move= 2.0
y_move= 1.0
completed=1
value = -0.1
import rospy 
from sensor_msgs.msg import Image
from grid_arm_lite.msg import location
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
import cv2
import matplotlib.pyplot as plt
out = [0,0,0]
bridge = CvBridge()
box = []
pub_x= rospy.Publisher('/grid/x_plotter_controller/command', Float64, queue_size=10)
pub_y= rospy.Publisher('/grid/y_plotter_controller/command', Float64, queue_size=10)
def callback1(data):
    global box
    global x_move
    global y_move
    global value
    global completed
    global out
    p = data.x + data.y + data.width + data.height
    if p != 0:
        box.append(data.x)
        box.append(data.y)
        box.append(data.width)
        box.append(data.height)
        completed = 0
    else:
        if x_move<4 and completed :
            if abs(y_move) > 1.0:
                x_move += 0.6
                value = -value
            y_move = y_move + value
            pub_x.publish(x_move)
            pub_y.publish(y_move)
def callback2(data):
    if box !=[]:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
        xp = box[0]
        yp = box[1]
        x = 1.38
        y = (-xp+320)*(1.491/525)
        z = (-yp+240)*(1.491/525)
        completed=0
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("location", location, callback1, queue_size=1)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback2, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
