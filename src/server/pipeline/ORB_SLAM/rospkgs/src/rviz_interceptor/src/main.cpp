#include <iostream>
#include <ros/ros.h>
#include "RvizInterceptor.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "RvizInterceptor");
    
    RvizInterceptor rv;

    ros::spin();

    return 0;
}
