#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cstdlib>
#include "BaseWrapper.hpp"

using std::placeholders::_1;

class BaseDriver : public rclcpp::Node
{
public:
    BaseDriver() : Node("base_driver")
    {
        base_ = std::make_shared<BaseWrapper>();
        
        // 1. Velocity Command
        sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&BaseDriver::cmd_vel_callback, this, _1));
            
        // 2. Lay Down (DAMP - Collapse)
        sub_laydown_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cmd_laydown", 10, std::bind(&BaseDriver::laydown_callback, this, _1));

        // 3. Stop Move (SDK Halt)
        sub_stop_move_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cmd_stop_move", 10, std::bind(&BaseDriver::stop_move_callback, this, _1));

        // 4. Stand Down (Controlled Sit)
        sub_stand_down_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cmd_stand_down", 10, std::bind(&BaseDriver::stand_down_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Base Driver Ready (Stop/StandDown Enabled).");
    }

    ~BaseDriver() {
        if(base_) base_->stop_move();
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        base_->move(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void laydown_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "CMD: DAMP (Collapse)");
            base_->damp(); 
        }
    }

    void stop_move_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            // Navigator spams this, so we don't log to keep console clean
            base_->stop_move(); 
        }
    }

    void stand_down_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "CMD: Stand Down (Controlled Sit)");
            base_->stand_down(); 
        }
    }

    std::shared_ptr<BaseWrapper> base_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_laydown_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_move_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stand_down_;
};

int main(int argc, char * argv[])
{
    // REMOVED THE PROBLEM-CAUSING CYCLONEDDS_URI XML BLOCK.
    // The required DDS configuration (like disabling shared memory) 
    // will now be handled by external environment variables (RMW_IMPLEMENTATION).
    
    // NOTE: If you are running on the Go2, the following line might be necessary 
    // to force a reliable network interface, though external environment variables 
    // are the primary fix for the crash.
    // setenv("CYCLONEDDS_URI", "<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>", 1);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseDriver>());
    rclcpp::shutdown();
    return 0;
}