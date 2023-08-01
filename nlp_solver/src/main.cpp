#include "../include/nlp_solver.hpp"

namespace vtec{

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nlp_node");
    solveNLP sol;
    sol.spin();
    std::cout << "\next program\n" << std::endl;
    return 0;
}

} // end vtec namespace