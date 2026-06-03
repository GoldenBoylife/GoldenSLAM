#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

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

    /*GoldenSlamApp이 아래 두개 클래스의 수명도 관리함*/
    std::unique_ptr<SlamCore>  slamCore_;
    std::shared_ptr<RosBridge> rosBridge_;
};
