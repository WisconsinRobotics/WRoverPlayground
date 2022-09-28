#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int8

def read_keys(msg):
    print(msg)
    throttle = (msg.data >> 3) - (msg.data >> 2 & 1)
    wheel = (msg.data >> 1 & 1) - (msg.data & 1)
    wheel_pub.publish(Float64(wheel))
    throttle_pub.publish(Float64(throttle))
    left = max(min((throttle + wheel), 1), -1)
    right = max(min((throttle - wheel), 1), -1)
    left_pub.publish(Float64(left))
    right_pub.publish(Float64(right))

rospy.init_node('controller')
print('load')
left_pub = rospy.Publisher('rover/cmd/left', Float64, queue_size=10)
right_pub = rospy.Publisher('rover/cmd/right', Float64, queue_size=10)
wheel_pub = rospy.Publisher('wheel', Float64, queue_size=10)
throttle_pub = rospy.Publisher('throttle', Float64, queue_size=10)

print('subs')
rospy.Subscriber('gui/keys', Int8, read_keys, queue_size=10)
rospy.spin()
print('spin')