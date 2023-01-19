#include <ros/ros.h>
#include "odometry_driver.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_driver");

    OdometryDriverRefactor odom_dr;
    odom_dr.run();

    return 0;
}