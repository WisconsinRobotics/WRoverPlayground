import rospy
from std_msgs.msg import String

QUEUE_SIZE = 10
RATE = 10

def callback(data):
    rospy.loginfo('I heard: [%s]', data.data)

def main():
    rospy.init_node('PubSubDemoPY', anonymous=True)
    
    pub = rospy.Publisher('chatter', String, queue_size=QUEUE_SIZE)
    rospy.Subscriber('chatter', String, callback)

    count = 0
    rate = rospy.Rate(RATE) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % count
        rospy.loginfo(hello_str)

        pub.publish(hello_str)

        rate.sleep()
        count += 1

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
