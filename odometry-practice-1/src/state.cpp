#include "state.h"
#include "logger.h"

State::State(HAL *_hal, const double start_x, const double start_y, const double _start_angle, const int _update_freq): hal(_hal), start_angle(_start_angle), update_freq(_update_freq) {
    Logger *logger = Logger::getDefault();
	logger->log("State::State() -> state.cpp", FUNCTION_CALL);
    mutex.take(TIMEOUT_MAX);
    hal->get_imu().reset(true);
    x = start_x;
    y = start_y;
    hal->get_imu().set_rotation(0);
    mutex.give();
}

void State::update(){
    static double update_delay = 1000.0 / update_freq;
    static double wheel_radius = 35; //in mm
    mutex.take(TIMEOUT_MAX);
    angle = start_angle - (M_PI * (hal->get_imu().get_rotation() / 180.0));
    if(!isForward){
        angle += M_PI;
    }
    mutex.give();
    left_one_rpm = -hal->get_left_one().get_actual_velocity() * (3/5);
    left_two_rpm = -hal->get_left_two().get_actual_velocity() * (3/5);
    left_three_rpm = -hal->get_left_three().get_actual_velocity() * (3/5);
    if(hal->get_left_one().is_reversed()){
        left_one_rpm = -left_one_rpm;
    }
    if(hal->get_left_two().is_reversed()){
        left_two_rpm = -left_two_rpm;
    }
    if(hal->get_left_three().is_reversed()){
        left_three_rpm = -left_three_rpm;
    }
    right_one_rpm = -hal->get_right_one().get_actual_velocity();
    right_two_rpm = -hal->get_right_two().get_actual_velocity();
    right_three_rpm = -hal->get_right_three().get_actual_velocity();
    if(!hal->get_right_one().is_reversed()){
        right_one_rpm = -right_one_rpm;
    }
    if(!hal->get_right_two().is_reversed()){
        right_two_rpm = -right_two_rpm;
    }
    if(!hal->get_right_three().is_reversed()){
        right_three_rpm = -right_three_rpm;
    }
    left_rpm = left_one_rpm;
    right_rpm = right_one_rpm;
    if(!isForward){
        left_rpm = -left_rpm;
        right_rpm = -right_rpm;
    }
    average_rpm = (left_rpm + right_rpm) / 2;
    mutex.take(TIMEOUT_MAX);
    velocity = average_rpm * 0.5 * M_PI * wheel_radius / 60.0;
    x += cos(angle) * velocity * (update_delay / 1000.0);
    y += sin(angle) * velocity * (update_delay / 1000.0);
    mutex.give();
}

