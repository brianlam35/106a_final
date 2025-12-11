#ifndef BASE_WRAPPER_HPP
#define BASE_WRAPPER_HPP

class BaseWrapper {
public:
    BaseWrapper();
    ~BaseWrapper();

    void move(float vx, float vy, float vyaw);
    void stop();
    
    // New function for controlled lowering
    void stand_down(); 

private:
    void* m_data = nullptr;
};

#endif // BASE_WRAPPER_HPP
