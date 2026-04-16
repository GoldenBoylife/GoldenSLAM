#include "app/golden_slam_app.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    std::cout << "start" << std::endl;
    GoldenSlamApp app(argc,argv);
    return app.run();
}