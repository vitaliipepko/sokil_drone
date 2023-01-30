#ifndef PID_H
#define PID_H

#include <ros/ros.h>


class PID
{
public:
    PID(float k_p, float k_i, float k_d, float min_output, float max_output, float integrator_min, float integrator_max, const std::string& name);

    void reset();
    void setIntegral(float integral);
    
    float getKI() const {return k_i_;}

    float update(float value, float target_value);
    
private:
    float k_p_;
    float k_i_;
    float k_d_;
    float min_output_;
    float max_output_;
    float integrator_min_;
    float integrator_max_;
    float integral_;
    float previous_error_;
    ros::Time previous_time_;
};

#endif