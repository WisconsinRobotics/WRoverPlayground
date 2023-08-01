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
    ros::init(argc, argv, "double_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<wrover_status_light::DoubleService>("double_service");

    wrover_status_light::DoubleService srv;
    srv.request.input = 1;

    ros::service::waitForService("double_service");
    while (ros::ok()) {
        if (client.call(srv)) {
            std::cout << srv.response.output << std::endl;
            srv.request.input = srv.response.output;
            ros::Duration(0.1).sleep();
        } else {

        }
    }
}