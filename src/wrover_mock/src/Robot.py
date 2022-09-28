#!/usr/bin/env python3

import rospy, std_msgs, geometry_msgs
from AbstractRobot import Robot
from geometry_msgs.msg import Pose2D

def set_left(msg): rover.setLeft(msg.data)

def set_right(msg): rover.setRight(msg.data)

def main():

    rospy.init_node('rover')
    
    rospy.Subscriber('rover/cmd/left',  std_msgs.msg.Float64, set_left, queue_size=10)
    rospy.Subscriber('rover/cmd/right',  std_msgs.msg.Float64, set_right, queue_size=10)

    left_enc_publisher = rospy.Publisher('rover/enc/left', std_msgs.msg.Float64, queue_size=10)
    right_enc_publisher = rospy.Publisher('rover/enc/left', std_msgs.msg.Float64, queue_size=10)
    gyro_publisher = rospy.Publisher('rover/gyro', std_msgs.msg.Float64, queue_size=10)
    pose_publisher = rospy.Publisher('rover/pose', Pose2D, queue_size=10) # top secret - do not subscribe!

    sleeper = rospy.Rate(hz)

    while not rospy.is_shutdown():

        rover.update()

        left_enc_publisher.publish(rover.getLeftDistance())
        right_enc_publisher.publish(rover.getRightDistance())
        gyro_publisher.publish(rover.getHeading())

        pose = Pose2D(rover.x, rover.y, rover.theta)
        # pose.data = (rover.x, rover.y, rover.theta)
        pose_publisher.publish(pose)

        sleeper.sleep()

hz = 20
rover = Robot(
    wheelBase = 50,
    maxVelocity = 1000/hz,
    acceleration = 10000,
)


if __name__ == '__main__':
    main()