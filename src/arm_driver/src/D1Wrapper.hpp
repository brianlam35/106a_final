#ifndef D1_WRAPPER_HPP
#define D1_WRAPPER_HPP

#include <vector>
#include <functional>

class D1Wrapper {
public:
    D1Wrapper();
    ~D1Wrapper();

    void enable_arm();
    void damp_arm();
    void send_command(double q0, double q1, double q2, double q3, double q4, double q5, double gripper);
    void set_feedback_callback(std::function<void(const std::vector<double>&)> callback);

private:
    // We use a raw void pointer to hide ALL implementation details from the header.
    // This prevents the "private struct" errors completely.
    void* m_data = nullptr;
};

#endif // D1_WRAPPER_HPP