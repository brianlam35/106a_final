#ifndef BASE_WRAPPER_HPP
#define BASE_WRAPPER_HPP

class BaseWrapper {
public:
    BaseWrapper();
    ~BaseWrapper();

    void move(float vx, float vy, float vyaw);
    
    // Stops the robot's movement (velocity = 0)
    void stop_move();
    
    // Emergency collapse (low stiffness)
    void damp(); 
    
    // Controlled sit (keeps motors active)
    void stand_down(); 

private:
    void* m_data = nullptr;
};

#endif // BASE_WRAPPER_HPP