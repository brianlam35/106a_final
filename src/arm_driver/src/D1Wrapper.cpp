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

// --- The Actual Data Struct (Hidden inside this file) ---
struct RealData {
    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> publisher;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> subscriber;
    std::function<void(const std::vector<double>&)> feedback_cb;
    int seq = 0;

    void process_msg(const void* msg) {
        if (!feedback_cb) return;
        const auto* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;
        std::vector<double> angles;
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

// Global Pointer for C-Style Callback
RealData* g_ptr = nullptr;

void GlobalHandler(const void* msg) {
    if (g_ptr) g_ptr->process_msg(msg);
}

// --- Class Implementation ---

D1Wrapper::D1Wrapper() {
    // Create the struct on the heap
    RealData* data = new RealData();
    m_data = data; // Store as void*
    g_ptr = data;  // Store global

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0);
        
        data->publisher = std::make_shared<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(CMD_TOPIC);
        data->publisher->InitChannel();

        data->subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(STATE_TOPIC);
        data->subscriber->InitChannel(GlobalHandler);
    } catch (...) {
        std::cerr << "[D1Wrapper] Init Failed!" << std::endl;
    }
}

D1Wrapper::~D1Wrapper() {
    g_ptr = nullptr;
    if (m_data) {
        delete static_cast<RealData*>(m_data);
        m_data = nullptr;
    }
}

void D1Wrapper::set_feedback_callback(std::function<void(const std::vector<double>&)> callback) {
    static_cast<RealData*>(m_data)->feedback_cb = callback;
}

void D1Wrapper::enable_arm() {
    RealData* d = static_cast<RealData*>(m_data);
    if (!d->publisher) return;
    d->seq++;
    std::stringstream ss;
    ss << "{\"seq\":" << d->seq << ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    std::cout << "[D1Wrapper] Sending Enable..." << std::endl;
    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = ss.str();
    d->publisher->Write(msg);
}

void D1Wrapper::damp_arm() {
    RealData* d = static_cast<RealData*>(m_data);
    if (!d->publisher) return;
    d->seq++;
    std::stringstream ss;
    ss << "{\"seq\":" << d->seq << ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":1}}";
    std::cout << "[D1Wrapper] Sending Relax..." << std::endl;
    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = ss.str();
    d->publisher->Write(msg);
}

void D1Wrapper::send_command(double q0, double q1, double q2, double q3, double q4, double q5, double gripper) {
    RealData* d = static_cast<RealData*>(m_data);
    if (!d->publisher) return;
    d->seq++;
    std::stringstream ss;
    ss << "{" << "\"seq\":" << d->seq << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
       << "\"angle0\":" << std::fixed << std::setprecision(2) << q0 << ","
       << "\"angle1\":" << q1 << "," << "\"angle2\":" << q2 << ","
       << "\"angle3\":" << q3 << "," << "\"angle4\":" << q4 << ","
       << "\"angle5\":" << q5 << "," << "\"angle6\":" << gripper << "}}";
    
    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = ss.str();
    d->publisher->Write(msg);
}