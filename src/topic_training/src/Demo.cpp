/* Include all headers of the ROS system we are using 
   separately. */
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"

/* Include the message type we will be receiving/subscribing. */
#include "std_msgs/Float64.h"

/* Include the message type we will be publishing. */
#include "std_msgs/String.h"

/* Include Stringstream to be able to manipulate the 
   data we are sending and receiving. (Optional: only 
   if the message is of type String)*/
#include <sstream>

/* Variable for message. */
float pi;

/**
 * This function will get called when a new message has 
 * arrived on the topic 'plot.' The variable pi will be 
   set as the data received from the Publisher 
   PlotDemo.cpp. The values given will be near continuous
   sine values in radians.
 * 
 * @param msg - the message received by the Subscriber, 
 *              sent by the Publisher
 */
void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
   pi = msg->data;
}

/**
 * The main method that contains the creation of the 
 * node, Publisher, Subcriber, and topic. It also 
 * describes what we will do with the data/message.
 */
int main(int argc, char **argv)
{
   /* Avoid unexplained literal numbers in code. */
   constexpr int QUEUE_SIZE = 1;
   constexpr int FREQUENCY = 10;

   /* Set up the ROS topic. */
   ros::init(argc, argv, "Demo");
   ros::NodeHandle nhandle;
   ros::Subscriber sub = nhandle.subscribe("plot", QUEUE_SIZE, chatterCallback);
   ros::Publisher pub = nhandle.advertise<std_msgs::String>("chatter", QUEUE_SIZE);

   /* Specify the frequency we would like to loop at 
      (10Hz). */
   ros::Rate loop_rate(FREQUENCY);

   /* The function ok() will return false if a SIGINT is 
      received (Ctrl-C), if a shutdown occurs, or if 
      another issue arises. Also, loops waiting for 
      'actual time' should always be conditioned on 
      ros::ok.
      
      (Optional: a while loop is not required for
      Publishers. Only used in this case to show the 
      continuous timing of publishing messages.) */
   while (ros::ok())
   {
      /* Since we are publishing a message with type 
         String, we will be following the next few steps 
         using Stringstream. This will not be the case 
         if our message type is different. The basic 
         idea is the same regardless of message type 
         though.
         
         Declare and initialize the Stringstream with 
         the String (+count) we want to publish. Then, 
         set the data of the message to be the stream 
         we just created. */
      std_msgs::String msg;
      std::stringstream sstream;
      sstream << "currently at " << pi;
      msg.data = sstream.str();

      /* Print out the message to the terminal. */
      ROS_INFO("%s", msg.data.c_str());

      /* Publish the message. */
      pub.publish(msg);

      ros::spinOnce();

      /* Use the function sleep() for the time remaining 
         until we reach our 10Hz publish rate. (Optional: 
         only used because we are looping.)*/
      loop_rate.sleep();
   }  

   return 0;
}
