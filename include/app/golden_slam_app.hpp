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
    int argc_;
    char** argv_;

    /*이 클래스에서 아래 클래스 2개의 수명을 관리할 것임*/
     std::unique_ptr<SlamCore> slamCore_;
     std::shared_ptr<RosBridge> rosBridge_;

};