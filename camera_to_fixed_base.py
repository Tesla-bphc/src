#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs


def transform_pose(input_pose, from_frame, to_frame):

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(3))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise