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
        
        sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&BaseDriver::cmd_vel_callback, this, _1));
            
        sub_laydown_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cmd_laydown", 10, std::bind(&BaseDriver::laydown_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Base Driver Ready.");
    }

    ~BaseDriver() {
        if(base_) base_->stop();
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        base_->move(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void laydown_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Lowering robot safely...");
            base_->stand_down();
        }
    }

    std::shared_ptr<BaseWrapper> base_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_laydown_;
};

int main(int argc, char * argv[])
{
    setenv("CYCLONEDDS_URI", "<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>", 1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseDriver>());
    rclcpp::shutdown();
    return 0;
}
