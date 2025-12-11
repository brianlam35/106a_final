#include "BaseWrapper.hpp"
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <iostream>
#include <memory>

struct RealBase {
    std::shared_ptr<unitree::robot::go2::SportClient> client;
};

BaseWrapper::BaseWrapper() {
    RealBase* data = new RealBase();
    m_data = data;

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0);
        data->client = std::make_shared<unitree::robot::go2::SportClient>();
        data->client->SetTimeout(10.0f);
        data->client->Init();
        std::cout << "[BaseWrapper] Connected!" << std::endl;
    } catch (...) {
        std::cerr << "[BaseWrapper] Failed!" << std::endl;
    }
}

BaseWrapper::~BaseWrapper() {
    if (m_data) delete static_cast<RealBase*>(m_data);
}

void BaseWrapper::stop() {
    RealBase* d = static_cast<RealBase*>(m_data);
    if(d->client) d->client->StopMove();
}

void BaseWrapper::move(float vx, float vy, float vyaw) {
    RealBase* d = static_cast<RealBase*>(m_data);
    if(d->client) d->client->Move(vx, vy, vyaw);
}

// --- NEW FUNCTION ---
void BaseWrapper::stand_down() {
    RealBase* d = static_cast<RealBase*>(m_data);
    if(d->client) {
        // This is the controlled "sit/lie down" command
        d->client->StandDown(); 
    }
}
