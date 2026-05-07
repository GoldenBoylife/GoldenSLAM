#include "app/golden_slam_app.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    GoldenSlamApp app(argc,argv);
    return app.run();
}