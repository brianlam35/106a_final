#ifndef D1_WRAPPER_HPP
#define D1_WRAPPER_HPP

#include <memory>
#include <vector>
#include <functional>

class D1Wrapper {
public:
    D1Wrapper();
    ~D1Wrapper();

    void enable_arm();
    void damp_arm();
    void send_command(double q0, double q1, double q2, double q3, double q4, double q5, double gripper);

    // New Function: Let the driver register a function to handle incoming angles
    void set_feedback_callback(std::function<void(const std::vector<double>&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // D1_WRAPPER_HPP