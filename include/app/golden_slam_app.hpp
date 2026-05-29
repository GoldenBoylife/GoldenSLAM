#pragma once

#include <memory>

class SlamCore;
class RosBridge;

class GoldenSlamApp
{
public:
    GoldenSlamApp(int argc, char** argv);
    ~GoldenSlamApp();

    int run();

private:
    int    argc_;
    char** argv_;

    std::unique_ptr<SlamCore>  slamCore_;
    std::shared_ptr<RosBridge> rosBridge_;
};
