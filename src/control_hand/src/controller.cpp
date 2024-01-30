/////////////////////////////////////////////////////////////////////////////////////////
// Program main

#include "control_hand/can_communicator.h"
#include "control_hand/RockScissorsPaper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_hand/srv/move.hpp"

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

        move_srv = create_service<control_hand::srv::Move>(
        "~/move",
        std::bind(
            &controller::move_callback,
            this, std::placeholders::_1,
            std::placeholders::_2));
    }
private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;
    rclcpp::Service<control_hand::srv::Move>::SharedPtr move_srv;
    
    void reset_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        response->success = true;
        response->message = "Resetting the hand";
        reset();        
    }

    void move_callback(
        const control_hand::srv::Move::Request::SharedPtr request,
        control_hand::srv::Move::Response::SharedPtr response)
    {
        response->success = true;
        switch (request->config_idx)
        {
            case 0:
                MotionRock();
                break;
            
            case 1:
                MotionScissors();
                break;
            
            case 2:
                MotionPaper();
                break;
            
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid configuration");
                response->success = false;
                break;
        }
    }


};

int main(int argc, char *argv[])
{
    CreateBHandAlgorithm();
    OpenCAN();
    memset(&vars, 0, sizeof(vars));
    memset(q, 0, sizeof(q));
    memset(q_des, 0, sizeof(q_des));
    memset(tau_des, 0, sizeof(tau_des));
    memset(cur_des, 0, sizeof(cur_des));
    curTime = 0.0;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller>());
    rclcpp::shutdown();
    CloseCAN();
    DestroyBHandAlgorithm();     
    return 0;
}

