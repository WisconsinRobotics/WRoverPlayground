#include "ros/ros.h"
#include "ros/init.h"
#include "ros/node_handle.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "double_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<wrover_status_light::DoubleService>("double_service");

    
}