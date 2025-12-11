#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // Standard ROS message for joints
#include <cmath>
#include <thread>
#include <cstdlib>
#include "D1Wrapper.hpp"

using std::placeholders::_1;

class ServoDriver : public rclcpp::Node
{
public:
    ServoDriver() : Node("servo_driver")
    {
        // 1. Setup Wrapper
        d1_ = std::make_shared<D1Wrapper>();
        
        // 2. Setup Feedback Callback
        // When wrapper gets data, it calls this lambda function
        d1_->set_feedback_callback([this](const std::vector<double>& angles) {
            this->publish_state(angles);
        });

        // 3. Setup Pub/Sub
        pub_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_joint_states", 10);
        
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/arm_joint_commands", 10, std::bind(&ServoDriver::topic_callback, this, _1));
        
        // 4. Enable Arm
        RCLCPP_INFO(this->get_logger(), "Enabling Arm...");
        rclcpp::sleep_for(std::chrono::seconds(1)); 
        d1_->enable_arm();
    }

    ~ServoDriver() {
        if(d1_) d1_->damp_arm();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

private:
    void publish_state(const std::vector<double>& angles_deg) {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};
        
        // Convert Degrees (Unitree) -> Radians (ROS)
        for(double deg : angles_deg) {
            msg.position.push_back(deg * (M_PI / 180.0));
        }
        
        pub_state_->publish(msg);
    }

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;
        double q[6];
        for(int i=0; i<6; i++) q[i] = msg->data[i] * (180.0 / M_PI);
        double gripper = (msg->data.size() >= 7) ? msg->data[6] * (180.0 / M_PI) : 0.0;
        d1_->send_command(q[0], q[1], q[2], q[3], q[4], q[5], gripper);
    }

    std::shared_ptr<D1Wrapper> d1_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_state_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    setenv("CYCLONEDDS_URI", "<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>", 1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoDriver>());
    rclcpp::shutdown();
    return 0;
}