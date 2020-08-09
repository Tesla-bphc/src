#!/usr/bin/env python
completed = 0
import rospy
import time
from grid_arm_full_24.msg import location
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from camera_to_fixed_base import transform_pose
p=Pose()
pub_z                                           = rospy.Publisher('/grid/z_plotter_controller/command', Float64, queue_size=10)
pub_x                                           = rospy.Publisher('/grid/x_plotter_controller/command', Float64, queue_size=10)
pub_y                                           = rospy.Publisher('/grid/y_plotter_controller/command', Float64, queue_size=10)
pub_joint_RR__GR                                = rospy.Publisher('/grid/joint_RR__GR_controller/command', Float64, queue_size=10)
pub_joint_TR__RR                                = rospy.Publisher('/grid/joint_TR__RR_controller/command', Float64, queue_size=10)
pub_joint_gripper_base__gripper_gear_l          = rospy.Publisher('/grid/joint_gripper_base__gripper_gear_l_controller/command', Float64, queue_size=10)
pub_joint_gripper_base__gripper_gear_r          = rospy.Publisher('/grid/joint_gripper_base__gripper_gear_r_controller/command', Float64, queue_size=10)
pub_joint_gripper_base__gripper_support_l       = rospy.Publisher('/grid/joint_gripper_base__gripper_support_l_controller/command', Float64, queue_size=10)
pub_joint_gripper_base__gripper_support_r       = rospy.Publisher('/grid/joint_gripper_base__gripper_support_r_controller/command', Float64, queue_size=10)
pub_joint_gripper_gear_l__gripper_main_l        = rospy.Publisher('/grid/joint_gripper_gear_l__gripper_main_l_controller/command', Float64, queue_size=10)
pub_joint_gripper_gear_r__gripper_main_r_back   = rospy.Publisher('/grid/joint_gripper_gear_r__gripper_main_r_back_controller/command', Float64, queue_size=10)
pub_joint_gripper_gear_r__gripper_main_r_front  = rospy.Publisher('/grid/joint_gripper_gear_r__gripper_main_r_front_controller/command', Float64, queue_size=10)
pub_joint_z_plotter__TR                         = rospy.Publisher('/grid/joint_z_plotter__TR_controller/command', Float64, queue_size=10)
def hold(value):
    pub_x.publish(value)
    time.sleep(2)
    pub_joint_z_plotter__TR.publish(-3.14)
    pub_joint_RR__GR.publish(-1.4)
    pub_joint_TR__RR.publish(0)
    pub_joint_gripper_base__gripper_gear_l.publish(-0.64)        
    pub_joint_gripper_base__gripper_gear_r.publish(-0.57)
    pub_joint_gripper_base__gripper_support_l.publish(+0.62)
    pub_joint_gripper_base__gripper_support_r.publish(-0.69)       
    pub_joint_gripper_gear_l__gripper_main_l.publish(0.34)
    pub_joint_gripper_gear_r__gripper_main_r_back.publish(0.34)
    pub_joint_gripper_gear_r__gripper_main_r_front.publish(0.34)
rospy.init_node('robot_controller', anonymous=True)
def prehold():
    pub_joint_z_plotter__TR.publish(-3.14)
    time.sleep(1)
    pub_joint_RR__GR.publish(-1.4)
    pub_joint_TR__RR.publish(0)
    pub_joint_gripper_base__gripper_gear_l.publish(0)        
    pub_joint_gripper_base__gripper_gear_r.publish(0)
    pub_joint_gripper_base__gripper_support_l.publish(0)
    pub_joint_gripper_base__gripper_support_r.publish(0)       
    pub_joint_gripper_gear_l__gripper_main_l.publish(0)
    pub_joint_gripper_gear_r__gripper_main_r_back.publish(0)
    pub_joint_gripper_gear_r__gripper_main_r_front.publish(0)
rospy.init_node('robot_controller', anonymous=True)


def home():
    time.sleep(2)
    pub_joint_z_plotter__TR.publish(-1.57)
    time.sleep(1)
    pub_joint_RR__GR.publish(1.57)
    pub_joint_TR__RR.publish(-1.57)
    pub_joint_gripper_base__gripper_gear_l.publish(0)        
    pub_joint_gripper_base__gripper_gear_r.publish(0)
    pub_joint_gripper_base__gripper_support_l.publish(0)
    pub_joint_gripper_base__gripper_support_r.publish(0)       
    pub_joint_gripper_gear_l__gripper_main_l.publish(0)
    pub_joint_gripper_gear_r__gripper_main_r_back.publish(0)
    pub_joint_gripper_gear_r__gripper_main_r_front.publish(0)
rospy.init_node('robot_controller', anonymous=True)
home()
def callback(data):
    global completed
    p = data.x + data.y + data.width + data.height
    if p != 0:
        xp = data.x
        yp = data.y
        x = 1.41
        y = (-xp+320)*(1.41/555)
        z = (-yp+240)*(1.41/550)
        d = Pose()
        d.position.x = x
        d.position.y = y
        d.position.z = z
        d.orientation.x = 0
        d.orientation.y = 0
        d.orientation.z = 0
        d.orientation.w = 0
        p = transform_pose(d, "camera_link", "fixed_base")
        pub_y.publish(-(p.position.y)+0.16)
        pub_x.publish(p.position.x+1.2)
        print(p.position.y)
        print(p.position.x)
        prehold()
        time.sleep(1)
        hold(p.position.x+1.45)
        time.sleep(2)
        pub_x.publish(1)
        time.sleep(1)
        pub_z.publish(1.0)
rospy.Subscriber("location", location, callback, queue_size=1)
rospy.spin()