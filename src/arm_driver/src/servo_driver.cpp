#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

// Unitree SDK2 Headers
#include <unitree/robot/channel/channel_publisher.hpp>
#include "ArmString_.hpp" // We will make CMake find this

#define TOPIC "rt/arm_Command"

using std::placeholders::_1;

class ServoDriver : public rclcpp::Node
{
public:
    ServoDriver() : Node("servo_driver")
    {
        // 1. Initialize Unitree Channel
        RCLCPP_INFO(this->get_logger(), "Initializing Unitree DDS Channel...");
        try {
            unitree::robot::ChannelFactory::Instance()->Init(0);
            publisher_ = std::make_shared<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(TOPIC);
            publisher_->InitChannel();
            RCLCPP_INFO(this->get_logger(), "Unitree Channel Ready!");
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init Unitree Channel. Is unitree_sdk2 installed?");
        }

        // 2. Subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/arm_joint_commands", 10, std::bind(&ServoDriver::topic_callback, this, _1));
            
        // Move to start pose (Optional: You can trigger this manually via topic if preferred)
        // send_command(0, 0, 0, 0, 0, 0); 
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 6) return;

        // ROS uses Radians, Unitree JSON uses Degrees. Convert!
        double q[6];
        for(int i=0; i<6; i++) {
            q[i] = msg->data[i] * (180.0 / M_PI);
        }

        // Send to Robot
        send_command(q[0], q[1], q[2], q[3], q[4], q[5]);
    }

    void send_command(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        static int seq = 0;
        seq++;

        // Construct the JSON String manually
        // Format: {"seq":X,"address":1,"funcode":2,"data":{"mode":1,"angle0":...}}
        std::stringstream ss;
        ss << "{"
           << "\"seq\":" << seq << ","
           << "\"address\":1,"
           << "\"funcode\":2,"  // 2 = Multiple Joint Control
           << "\"data\":{"
           << "\"mode\":1,"     // 1 = Position Mode
           << "\"angle0\":" << std::fixed << std::setprecision(2) << q0 << ","
           << "\"angle1\":" << q1 << ","
           << "\"angle2\":" << q2 << ","
           << "\"angle3\":" << q3 << ","
           << "\"angle4\":" << q4 << ","
           << "\"angle5\":" << q5 << ","
           << "\"angle6\":0"    // Gripper (Joint 7) - Add logic if needed later
           << "}}";

        // Create and Send Message
        unitree_arm::msg::dds_::ArmString_ dds_msg;
        dds_msg.data_() = ss.str();
        
        if (publisher_) {
            publisher_->Write(dds_msg);
        }
    }

    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoDriver>());
    rclcpp::shutdown();
    return 0;
}