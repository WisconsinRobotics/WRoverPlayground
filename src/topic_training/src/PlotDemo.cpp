#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/publisher.h"

/* Include the message type we will be publishing. */
#include "std_msgs/Float64.h"
/* Include math functions (such as sin). */
#include "cmath"

constexpr int QUEUE_SIZE = 1;
constexpr int FREQUENCY = 10;
/* The rate of incrementation. */
constexpr float increment = 0.1;

auto main(int argc, char **argv) -> int
{
   ros::init(argc, argv, "PlotDemo");
   ros::NodeHandle nhandle;
   ros::Publisher pub = nhandle.advertise<std_msgs::Float64>("chatter", QUEUE_SIZE);

   ros::Rate loop_rate(FREQUENCY);

   /* Start from 0 radians. */
   float count = 0;
   while (ros::ok())
   {
      std_msgs::Float64 msg;

      /* The function sinf() takes in radians as the 
         parameter and returns the sine value as a 
         float. */
      float sineValu = sinf(count);
      msg.data = sineValu;
      pub.publish(msg);
    
      ros::spinOnce();
      loop_rate.sleep();
      /* Increment by 0.1 radians to have a smooth 
         sine-wave graph. */
      count += increment;
   }  

   return 0;
}
