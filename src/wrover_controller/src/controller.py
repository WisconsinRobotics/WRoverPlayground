#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D


def handle_pose(msg):
    y = msg.y

    if y < 50:
        left_pub.publish(Float64(1))
        right_pub.publish(Float64(1))
    else:
        left_pub.publish(Float64(0))
        right_pub.publish(Float64(0))



rospy.init_node("controller")

left_pub = rospy.Publisher("/rover/cmd/left", Float64, queue_size=10)
right_pub = rospy.Publisher("/rover/cmd/right", Float64, queue_size=10)
has_stopped = False

rospy.Subscriber("/rover/pose", Pose2D, handle_pose)
rospy.spin()