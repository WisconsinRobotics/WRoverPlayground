import rospy
''' Import the message type we will be publishing. '''
from std_msgs.msg import Float64
''' Import math functions (such as sin). '''
import math

QUEUE_SIZE = 10
RATE = 10

def main():
    rospy.init_node('PlotDemo', anonymous=True)
    pub = rospy.Publisher('chatter', Float64, queue_size=QUEUE_SIZE)
    rate = rospy.Rate(RATE)

    ''' Start from 0 radians. '''
    count = 0
    while not rospy.is_shutdown():
        ''' The function sin() takes in radians as the 
            parameter. '''
        sine_value = math.sin(count)
        pub.publish(sine_value)
        rate.sleep()
        ''' Increment by 0.1 radians to have a smooth 
            sine-wave graph. '''
        count += 0.1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass