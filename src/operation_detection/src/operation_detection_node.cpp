#include "operation_detection_core.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "operation_detection");

    OperationDetector od;
    od.run();

    return 0;
}