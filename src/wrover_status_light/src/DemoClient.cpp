#include "ros/duration.h"
#include "ros/ros.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/service.h"
#include <iostream>
#include <iterator>
#include <ostream>
#include <wrover_status_light/DoubleService.h>

int main(int argc, char** argv) {
    /*
    Creates a new node called "double_client" and a new NodeHandle reference. NodeHandles provide 
    functions for creating, managing, and interacting with ROS nodes.
    */
    ros::init(argc, argv, "double_client");
    ros::NodeHandle nh;

    /*
    Creates a new client instance using the NodeHandle. Here it specifies the service type 
    "wrover_status_light::DoubleService" and service name "double_service".
    */
    ros::ServiceClient client = nh.serviceClient<wrover_status_light::DoubleService>("double_service");

    /*
    Initializes various local variables. rate specifies the frequency we would want to run at, and 
    srv is an instance to the service (which envelopes accessing the request and response fields).
    */
    ros::Duration rate = ros::Duration(0.1);
    wrover_status_light::DoubleService srv;
    srv.request.input = 1;

    /*
    This is blocking call that waits until a service called "double_service" is created and ready.
    */
    ros::service::waitForService("double_service");

    /*
    The funciton ros::ok() checks if the module is still running or not. 
    */
    while (ros::ok()) {

        /*
        Calls the service using the client instance created earlier. The call returns true if the 
        operation is successful and false otherwise.
        */
        if (client.call(srv)) {
            std::cout << srv.response.output << std::endl;
            srv.request.input = srv.response.output;

            /*
            Waits for 100ms before resuming the program. 
            */
            rate.sleep();
        } else {
            /*
            This code block is ran when the service call exits unsuccessfully. In this example it 
            is not populated for simplicity. 
            */
        }
    }
}