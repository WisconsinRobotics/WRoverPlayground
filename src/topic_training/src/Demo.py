''' Import rospy to use ROS with python. '''
import rospy
''' Import the message type we will be publishing. '''
from std_msgs.msg import String
from std_msgs.msg import Float64

''' Constant variables to be used below. '''
QUEUE_SIZE = 1
RATE = 10

''' Variable for message. '''
pi = 0.0

'''
This function will get called when a new message has 
arrived on the topic 'plot.' The variable pi will be 
set as the data received from the Publisher PlotDemo.py. 
The values given will be near continuous sine values in 
radians.
'''
def callback(data: Float64) -> None:
    global pi 
    pi = data.data

''' 
The main method that contains the creation of the node, 
Publisher, Subcriber, and topic. It also describes what 
we will do with the data/message. 
'''
def main():
    ''' Set up the ROS topic. '''
    rospy.init_node('Demo', anonymous=True)
    rospy.Subscriber('plot', Float64, callback)
    pub = rospy.Publisher('chatter', String, queue_size=QUEUE_SIZE)

    ''' Specify the frequency we would like to loop at 
        (10Hz). '''
    rate = rospy.Rate(RATE)

    ''' The function is_shutdown() checks if the program 
        should exit (Ctrl-C or another issue).
        
        (Optional: a while loop is not required for
        Publishers. Only used in this case to show the 
        continuous timing of publishing messages.) '''
    while not rospy.is_shutdown():
        ''' Create the message and print out the message 
            to the terminal. '''
        pi_msg = "currently at %s" % pi
        rospy.loginfo(pi_msg)

        ''' Publish the message. '''
        pub.publish(pi_msg)

        ''' Use the function sleep() for the time 
            remaining until we reach our 10Hz publish 
            rate. (Optional: only used because we are 
            looping.)'''
        rate.sleep()

'''
Run main. If a ROSInterruptException is thrown by sleep() 
when a SIGINT (Ctrl-C) is detected or if the node is 
shutdown. This keeps us from continuing to execute code 
after sleep().
'''
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
