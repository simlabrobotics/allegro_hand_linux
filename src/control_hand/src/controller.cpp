/////////////////////////////////////////////////////////////////////////////////////////
// Program main

#include "control_hand/can_communicator.h"
#include "control_hand/RockScissorsPaper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class controller : public rclcpp::Node
{
public:
    controller(): Node("controller")
    {
        reset_srv = create_service<std_srvs::srv::Trigger>(
        "~/reset",
        std::bind(
            &controller::reset_callback,
            this, std::placeholders::_1,
            std::placeholders::_2));
    }
private:


    void reset_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        response->success = true;
        response->message = "Resetting the hand";
        memset(&vars, 0, sizeof(vars));
        memset(q, 0, sizeof(q));
        memset(q_des, 0, sizeof(q_des));
        memset(tau_des, 0, sizeof(tau_des));
        memset(cur_des, 0, sizeof(cur_des));
        curTime = 0.0;
        reset();        
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;
};

int main(int argc, char *argv[])
{
    CreateBHandAlgorithm();
    OpenCAN();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller>());
    rclcpp::shutdown();
    CloseCAN();
    DestroyBHandAlgorithm();     
    return 0;
}


// int main(int argc, TCHAR* argv[])
// {
//     PrintInstruction();

//     memset(&vars, 0, sizeof(vars));
//     memset(q, 0, sizeof(q));
//     memset(q_des, 0, sizeof(q_des));
//     memset(tau_des, 0, sizeof(tau_des));
//     memset(cur_des, 0, sizeof(cur_des));
//     curTime = 0.0;

//     if (CreateBHandAlgorithm() && OpenCAN())
//         MainLoop();

//     CloseCAN();
//     DestroyBHandAlgorithm();

//     return 0;
// }