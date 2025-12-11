#include "D1Wrapper.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <iostream>

#define CMD_TOPIC "rt/arm_Command"
#define STATE_TOPIC "current_servo_angle"

// --- Implementation Struct ---
struct D1Wrapper::Impl {
    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> publisher;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> subscriber;
    std::function<void(const std::vector<double>&)> feedback_cb;
    int seq = 0;
    
    void process_msg(const void* msg) {
        if (!feedback_cb) return;
        
        const auto* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;
        std::vector<double> angles;
        
        // Extract 6 Joints + Gripper
        angles.push_back(pm->servo0_data_());
        angles.push_back(pm->servo1_data_());
        angles.push_back(pm->servo2_data_());
        angles.push_back(pm->servo3_data_());
        angles.push_back(pm->servo4_data_());
        angles.push_back(pm->servo5_data_());
        angles.push_back(pm->servo6_data_());
        
        feedback_cb(angles);
    }
};

// --- Global Pointer Hack ---
// We use void* here to avoid the "private struct" compiler error
void* g_d1_impl_ptr = nullptr;

// The C-Style Handler that Unitree calls
void GlobalHandler(const void* msg) {
    if (g_d1_impl_ptr) {
        // Cast it back to the real type
        static_cast<D1Wrapper::Impl*>(g_d1_impl_ptr)->process_msg(msg);
    }
}

D1Wrapper::D1Wrapper() : impl_(new Impl()) {
    try {
        g_d1_impl_ptr = impl_.get(); // Point global ptr to this instance
        
        // Removed "eth0" here because we handle it in main() of servo_driver.cpp
        unitree::robot::ChannelFactory::Instance()->Init(0);
        
        // 1. Publisher
        impl_->publisher = std::make_shared<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(CMD_TOPIC);
        impl_->publisher->InitChannel();

        // 2. Subscriber
        impl_->subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(STATE_TOPIC);
        impl_->subscriber->InitChannel(GlobalHandler);

    } catch (...) {
        std::cerr << "[D1Wrapper] Init Failed!" << std::endl;
    }
}

D1Wrapper::~D1Wrapper() {
    g_d1_impl_ptr = nullptr;
}

void D1Wrapper::set_feedback_callback(std::function<void(const std::vector<double>&)> callback) {
    impl_->feedback_cb = callback;
}

void D1Wrapper::enable_arm() {
    if (!impl_->publisher) return;
    impl_->seq++;
    std::stringstream ss;
    ss << "{\"seq\":" << impl_->seq << ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    std::cout << "[D1Wrapper] Sending Enable..." << std::endl;
    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = ss.str();
    impl_->publisher->Write(msg);
}

void D1Wrapper::damp_arm() {
    if (!impl_->publisher) return;
    impl_->seq++;
    std::stringstream ss;
    ss << "{\"seq\":" << impl_->seq << ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":1}}";
    std::cout << "[D1Wrapper] Sending Relax..." << std::endl;
    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = ss.str();
    impl_->publisher->Write(msg);
}

void D1Wrapper::send_command(double q0, double q1, double q2, double q3, double q4, double q5, double gripper) {
    if (!impl_->publisher) return;
    impl_->seq++;
    std::stringstream ss;
    ss << "{" << "\"seq\":" << impl_->seq << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
       << "\"angle0\":" << std::fixed << std::setprecision(2) << q0 << ","
       << "\"angle1\":" << q1 << "," << "\"angle2\":" << q2 << ","
       << "\"angle3\":" << q3 << "," << "\"angle4\":" << q4 << ","
       << "\"angle5\":" << q5 << "," << "\"angle6\":" << gripper << "}}";
    
    // Uncomment to debug
    // std::cout << "[D1Wrapper] Sending Cmd..." << std::endl;

    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = ss.str();
    impl_->publisher->Write(msg);
}