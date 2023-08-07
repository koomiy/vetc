#include "../include/mpc_controller.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_node");
    mpc control;
    control.init();
    control.spin();
    std::cout << "\next program\n" << std::endl;
    return 0;
}
