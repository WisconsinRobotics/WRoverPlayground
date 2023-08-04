''' Import rospy to use ROS with python. '''
import rospy
''' Import the message type we will be publishing. '''
from std_msgs.msg import String

''' Constant variables to be used below. '''
QUEUE_SIZE = 10
RATE = 10

'''
This function will get called when a new message has 
arrived on the topic.
'''
def callback(data: String) -> None:
    ''' Print out the message to the terminal. '''
    rospy.loginfo('I heard: [%s]', data.data)

''' 
The main method that contains the creation of the node, 
Publisher, Subcriber, and topic. It also describes what 
we will do with the data/message. 
'''
def main():
    ''' Initialize ROS and specify the name of the node 
        (which is the name of the file in this case). '''
    rospy.init_node('Demo', anonymous=True)
    
    ''' Initialize the Publisher. State the name of the 
        topic ('chatter'), the message type we will be 
        publishing (String, which is actually in the 
        class std_msgs.msg.String), and the queue size 
        (1000). The queue size is the maximum number of 
        messages kept as a buffer before throwing away 
        old ones if we are publishing too quickly. '''
    pub = rospy.Publisher('chatter', String, queue_size=QUEUE_SIZE)

    ''' Initialize the Subscriber. State the name of the 
        topic ('chatter'), the message type we will be 
        publishing (String, which is, again, actually in 
        the class std_msgs.msg.String), and the function 
        that will be called when a message is received 
        (callBack(data)). '''
    rospy.Subscriber('chatter', String, callback)

    ''' Specify the frequency we would like to loop at 
        (10Hz). '''
    rate = rospy.Rate(RATE)

    ''' A counter of how many messages have been sent. 
        (Optional: only used for this example to show 
        the difference in messages) '''
    count = 0
    ''' (Optional: a while loop is not required for
        Publishers. Only used in this case to show the 
        continuous timing of publishing messages.) '''
    while not rospy.is_shutdown():
        ''' The function is_shutdown() checks if the program 
            should exit (Ctrl-C or another issue). '''
        
        ''' Create the message of type String. '''
        hello_str = "hello world %s" % count
        ''' Print out the message to the terminal. '''
        rospy.loginfo(hello_str)

        ''' Publish the message to the topic for the 
            Subscriber to receive. '''
        pub.publish(hello_str)

        ''' Use the function sleep() for the time 
            remaining until we reach our 10Hz publish 
            rate. '''
        rate.sleep()
        
        ''' Increment count. (Optional: as stated 
            before) '''
        count += 1

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
