#include "state.h"
#include "logger.h"

State::State(const int left_one_port, const int left_two_port, const int left_three_port, const int right_one_port, const int right_two_port, const int right_three_port, const int _imu_port, const double start_x, const double start_y, const double _start_angle, const int _update_freq): left_one(left_one_port), left_two(left_two_port), left_three(left_three_port), right_one(right_one_port), right_two(right_two_port), right_three(right_three_port), start_angle(_start_angle), update_freq(_update_freq), imu(_imu_port) {
    Logger *logger = Logger::getDefault();
	logger->log("State::State() -> state.cpp");
    mutex.take(TIMEOUT_MAX);
    imu.reset(true);
    x = start_x;
    y = start_y;
    imu.set_rotation(0);
    mutex.give();
}
void State::update(){
    double update_delay = 1000.0 / update_freq;
    double wheel_radius = 30; //in mm
    uint32_t *tstamp = new uint32_t(millis());
    mutex.take(TIMEOUT_MAX);
    angle = start_angle - (M_PI * (imu.get_rotation() / 180.0));
    mutex.give();
    left_one_rpm = -left_one.get_actual_velocity();
    left_two_rpm = -left_two.get_actual_velocity();
    left_three_rpm = -left_three.get_actual_velocity();
    if(left_one.is_reversed()){
        left_one_rpm = -left_one_rpm;
    }
    if(left_two.is_reversed()){
        left_two_rpm = -left_two_rpm;
    }
    if(left_three.is_reversed()){
        left_three_rpm = -left_three_rpm;
    }
    right_one_rpm = -right_one.get_actual_velocity();
    right_two_rpm = -right_two.get_actual_velocity();
    right_three_rpm = -right_three.get_actual_velocity();
    if(!right_one.is_reversed()){
        right_one_rpm = -right_one_rpm;
    }
    if(!right_two.is_reversed()){
        right_two_rpm = -right_two_rpm;
    }
    if(!right_three.is_reversed()){
        right_three_rpm = -right_three_rpm;
    }
    left_rpm = left_one_rpm;
    right_rpm = right_one_rpm;
    average_rpm = (left_rpm + right_rpm) / 2;
    mutex.take(TIMEOUT_MAX);
    velocity = average_rpm * 0.5 * M_PI * wheel_radius / 60.0;
    x += cos(angle) * velocity * (update_delay / 1000.0);
    y += sin(angle) * velocity * (update_delay / 1000.0);
    mutex.give();
    delete tstamp;
}

