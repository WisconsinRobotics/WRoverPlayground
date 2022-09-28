#!/usr/bin/env python3

import rospy, std_msgs
from AbstractRobot import Robot

def setLeft(msg): rover.setLeft(msg.data)

def setRight(msg): rover.setRight(msg.data)


def main():

    rospy.init_node('rover')
    
    rospy.Subscriber('rover/cmd/left',  std_msgs.msg.Float64, setLeft, queue_size=10)
    rospy.Subscriber('rover/cmd/right',  std_msgs.msg.Float64, setRight, queue_size=10)
    left_enc_publisher = rospy.Publisher('rover/enc/left', std_msgs.msg.Float64, queue_size=10)
    right_enc_publisher = rospy.Publisher('rover/enc/left', std_msgs.msg.Float64, queue_size=10)
    gyro_publisher = rospy.Publisher('rover/gyro', std_msgs.msg.Float64, queue_size=10)
    pose_publisher = rospy.Publisher('rover/pose', std_msgs.msg.Float64MultiArray, queue_size=10) # top secret - do not subscribe!

    sleeper = rospy.Rate(20)

    while not rospy.is_shutdown():
        rover.update()

        left_enc_publisher.publish(rover.getLeftDistance())
        right_enc_publisher.publish(rover.getRightDistance())
        gyro_publisher.publish(rover.getHeading())

        pose = std_msgs.msg.Float64MultiArray()
        pose.data = (rover.x, rover.y, rover.theta)
        pose_publisher.publish(pose)

        sleeper.sleep()

rover = Robot(
    wheelBase = 50,
    maxVelocity = 25,
    acceleration = 5,
)


if __name__ == '__main__':
    main()