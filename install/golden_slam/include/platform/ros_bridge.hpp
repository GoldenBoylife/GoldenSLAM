#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>




class SlamCore;

class RosBridge : public rclcpp::Node
{

public:
    explicit RosBridge(SlamCore* core);
    //명백한 이런 형태로만 선언

private:
    void onFrontendTimer();
    SlamCore* core_;
    //나는 소유자는 아니고, 누가 만든거 가져다 쓰겠다는 뜻
    // 진짜 주인은 GoldenSlamApp임

    rclcpp::TimerBase::SharedPtr frontend_timer_;

};