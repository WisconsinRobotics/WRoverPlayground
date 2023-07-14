#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include "std_msgs/String.h"

#include <sstream>

const int QUEUE_SIZE = 1000;
const int FREQUENCY = 10;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

auto main(int argc, char **argv) -> int
{
  ros::init(argc, argv, "PubSubDemo");

  ros::NodeHandle nhandle;

  ros::Publisher chatter_pub = nhandle.advertise<std_msgs::String>("chatter", QUEUE_SIZE);
  ros::Subscriber sub = nhandle.subscribe("chatter", QUEUE_SIZE, chatterCallback);

  ros::Rate loop_rate(FREQUENCY);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream sstream;
    sstream << "hello world " << count;
    msg.data = sstream.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }  

  ros::spin();

  return 0;
}
