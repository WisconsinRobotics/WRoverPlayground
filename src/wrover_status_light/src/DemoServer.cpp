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
    return res.output > 0;
}

int main(int argc, char** argv) {
    /*
    The ros::init() function sets up the node and needs to be called before using any other parts 
    of the ROS library. Here it takes in 3 arguments: argc, argv, and the name of this node. 
    */
    ros::init(argc, argv, "double_server");

    /*
    A NodeHandle is the main access point to communications with ROS in C++. Here it is used to 
    create a new server.
    */
    ros::NodeHandle nh;

    /*
    Creates a new server instance using the NodeHandle, with "double_service" as the server name 
    and "doubleServiceCallback" as the handler function. 
    */
    ros::ServiceServer service = nh.advertiseService("double_service", doubleServiceCallback);

    /*
    The ros::spin() function and other versions of spin() keeps the code running by entering a 
    loop that listens for incoming requests.
    */
    ros::spin();
    return 0;
}