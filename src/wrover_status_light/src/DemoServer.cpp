#include "ros/ros.h"
#include "ros/init.h"
#include <wrover_status_light/DoubleService.h>

bool addTwoIntsCallback(wrover_status_light::DoubleServiceRequest& req,
                        wrover_status_light::DoubleServiceResponse& res) {
    res.output = req.input * 2;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "double_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("double_service", addTwoIntsCallback);

    ros::spin();

    return 0;
}