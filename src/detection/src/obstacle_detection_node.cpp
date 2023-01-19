#include "obstacle_detection.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");

    ObstacleDetector od;
    od.run();

    return 0;
}