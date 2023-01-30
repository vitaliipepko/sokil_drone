#include <ros/ros.h>
#include <sokil_drone_controller/pid.h>


PID::PID(
    float k_p, 
    float k_i, 
    float k_d, 
    float min_output, 
    float max_output, 
    float integrator_min, 
    float integrator_max, 
    const std::string& name)
    : k_p_(k_p),
    k_i_(k_i),
    k_d_(k_d),
    min_output_(min_output),
    max_output_(max_output),
    integrator_min_(integrator_min),
    integrator_max_(integrator_max),
    integral_(0),
    previous_error_(0),
    previous_time_(ros::Time::now())
    {
    }

    PID::reset()
    {
        integral_ = 0;
        previous_error_ = 0;
        previous_time = rosTime::now();
    }

    PID::setIntegral(float integral)
    {
        integral_ = integral;
    }

    PID::update(float value, float target_value)
    {
        ros::Time time{ros::Time::now()};
        float dt{time.toSec() - previous_time_.toSec();
        float error{target_value - value}};
        integral_ += error * dt;
        integral_ = std::max(std::min(integral_, integrator_max_), integrator_min_);
        float p{k_p_ * error};
        float d{0};

        if(dt > 0)
        {
            d = k_d_ * (error - previous_error_) / dt;
        }

        float i{k_i_ * integral_};
        float output{p + i + d};
        previous_error = error;
        previous_time = time;

        return srd::max(std::min(output, max_output_), min_output_);
    }