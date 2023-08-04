/* Include all headers of the ROS system we are using 
   separately. */
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"

/* Include the message type we will be publishing. */
#include "std_msgs/String.h"

/* Include Stringstream to be able to manipulate the 
   data we are sending and receiving. (Optional: only 
   if the message is of type String)*/
#include <sstream>

/* Avoid unexplained literal numbers in code. */
constexpr int QUEUE_SIZE = 1;
constexpr int FREQUENCY = 10;

/**
 * This function will get called when a new message has 
 * arrived on the topic.
 * 
 * @param msg - the message received by the Subscriber, 
 *              sent by the Publisher
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
   /* Print out the message to the terminal. */
   ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * The main method that contains the creation of the 
 * node, Publisher, Subcriber, and topic. It also 
 * describes what we will do with the data/message.
 */
auto main(int argc, char **argv) -> int
{

   /* Initialize ROS and specify the name of the node 
      (which is the name of the file in this case). 
      Requires the parameters from the main function 
      header. */
   ros::init(argc, argv, "Demo");

   /* Create a handle for the node. This will do the 
      actually initialization of the node in the next 
      few lines. */
   ros::NodeHandle nhandle;

   /* Initialize the Publisher. The advertise() function 
      is how we tell ROS we want to publish on a topic. 
      State the message type we will be publishing 
      (std_msgs::String), the name of the topic 
      ("chatter"), and the queue size (1000). The queue 
      size is the maximum number of messages kept as a 
      buffer before throwing away old ones if we are 
      publishing too quickly. */
   ros::Publisher pub = nhandle.advertise<std_msgs::String>("chatter", QUEUE_SIZE);

   /* Initialize the Subscriber. The subscriber() function 
      is how we tell ROS we want to receive messages on 
      a specific topic. State the name of the topic 
      ("chatter"), the queue size (1000), and the function 
      that will be called when a message is received 
      (chatterCallBack(msg)). */
   ros::Subscriber sub = nhandle.subscribe("chatter", QUEUE_SIZE, chatterCallback);

   /* Specify the frequency we would like to loop at 
      (10Hz). */
   ros::Rate loop_rate(FREQUENCY);

   /* A counter of how many messages have been sent. 
      (Optional: only used for this example to show the 
      difference in messages) */
   int count = 0;
   /* (Optional: a while loop is not required for
      Publishers. Only used in this case to show the 
      continuous timing of publishing messages.) */
   while (ros::ok())
   /* The function ok() will return false if a SIGINT is 
      received (Ctrl-C), if a shutdown occurs, or if 
      another issue arises. Also, loops waiting for 
      'actual time' should always be conditioned on 
      ros::ok. */
   {
      /* Declare the message as the type we specified 
         when we initialized the Publisher 
         (std_msgs::String). */
      std_msgs::String msg;

      /* Since we are publishing a message with type 
         String, we will be following the next few steps 
         using Stringstream. This will not be the case if 
         our message type is different. The basic idea is 
         the same regardless of message type though.*/
    
      /* Declare the Stringstream. */
      std::stringstream sstream;
      /* Initialize the stream with the String (+count) 
         we want to publish. */
      sstream << "hello world " << count;
      /* Set the data of the message to be the stream we 
         just created. */
      msg.data = sstream.str();

      /* Print out the message to the terminal. */
      ROS_INFO("%s", msg.data.c_str());

      /* Publish the message to the topic for the 
         Subscriber to receive. */
      pub.publish(msg);
    
      /* The function spinOnce() allows us to have a 
         Publisher and Subscriber in the same file. If it 
         were not here, our callbacks would never get 
         called. (Optional: only required if Publisher 
         and Subscriber are separate) */
      ros::spinOnce();

      /* Use the function sleep() for the time remaining 
         until we reach our 10Hz publish rate. (Optional: 
         only used because we are looping.)*/
      loop_rate.sleep();

      /* Increment count. (Optional: as stated before) */
      ++count;
   }  

   return 0;
}
