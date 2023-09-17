#!/bin/python3

import rospy
from wrover_test.msg import HelloMsg

def main():
    '''
    A function that publishes "Hello World!" and a count value to a hello_world topic once per second
    '''
    rospy.init_node('hello', anonymous=True)
    pub = rospy.Publisher('hello_world', HelloMsg, queue_size=1)

    count = 0
    while not rospy.is_shutdown():
        pub.publish('Hello World!', count)
        count += 1
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
