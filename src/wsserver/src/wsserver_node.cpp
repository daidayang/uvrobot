#include "wsserver_core.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wsserver");

    WSServer server;
    server.run();

    return 0;
}