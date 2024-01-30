/////////////////////////////////////////////////////////////////////////////////////////
// Program main

#include "control_hand/can_communicator.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class controller : public rclcpp::Node
{
public:
    controller(): Node("controller")
    {
        // Create a publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&controller::timer_callback, this));
    }
private:

    

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller>());
    rclcpp::shutdown();
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