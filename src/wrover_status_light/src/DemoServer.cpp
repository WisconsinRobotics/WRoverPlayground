#include "ros/ros.h"
#include "ros/init.h"
#include <wrover_status_light/DoubleService.h>

/*
The handler function of the service. It will be called whenever this server receives a new 
request message. Handler functions are responsible for carrying out the actual logics of the 
service and send a response back to the client. Handler functions in C++ always takes in a 
reference to the request and a reference to the response as arguments.
*/
bool doubleServiceCallback(wrover_status_light::DoubleServiceRequest& req,
                        wrover_status_light::DoubleServiceResponse& res) {
    /*
    Here the handler functions doubles the input field for the requets and sets it as the output 
    field for the response.
    */
    res.output = req.input * 2;

    /*
    Since a reference to the response is already provided in the arguments, we don't need to 
    return it. Instead, handler functions in C++ return a boolean to convey if the operation is 
    successful or not. 
    */
    return true;
}

int main(int argc, char** argv) {
    /*
    Creates a new node called "double_server" and a new NodeHandle reference. NodeHandles provide 
    functions for creating, managing, and interacting with ROS nodes.
    */
    ros::init(argc, argv, "double_server");
    ros::NodeHandle nh;

    /*
    Creates a new server instance using the NodeHandle, with "double_service" as the server name 
    and "doubleServiceCallback" as the handler function. 
    */
    ros::ServiceServer service = nh.advertiseService("double_service", doubleServiceCallback);

    /*
    Keeps the code running by entering a loop that listens for incoming requests.
    */
    ros::spin();
    return 0;
}