#include "state_manager_core.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_manager");

    StateManagerCore smc;
    smc.run();

    return 0;
}
