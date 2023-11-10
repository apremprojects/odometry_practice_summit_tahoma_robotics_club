#include "state.h"

State::State(const int left_port, const int right_port, const int imu_port, const int _update_freq): left(left_port), right(right_port), imu(imu_port), update_freq(_update_freq){
    imu.reset(true);
    imu.set_rotation(0);
}

void State::update(){
    double update_delay = 1000.0 / update_freq;
    int wheel_radius = 30; //in mm
    uint32_t *tstamp = new uint32_t(millis());
    double left_rev = -1;
    if(!left.is_reversed()){
        left_rev = left.get_raw_position(tstamp) / 1800.0;
    } else{
        left_rev = -left.get_raw_position(tstamp) / 1800.0;
    }
    double right_rev = -1;
    if(right.is_reversed()){
        right_rev = right.get_raw_position(tstamp) / 1800.0;
    } else{
        right_rev = -right.get_raw_position(tstamp) / 1800.0;
    }
    angle = 2 * M_PI * (imu.get_rotation() / 360.0);
    double left_rpm = left.get_actual_velocity();
    if(left.is_reversed()){
        left_rpm = -left_rpm;
    }
    double right_rpm = right.get_actual_velocity();
    if(!right.is_reversed()){
        right_rpm = -right_rpm;
    }
    double average_rpm = (left_rpm + right_rpm) / 2;
    velocity = average_rpm * 0.5 * M_PI * wheel_radius / 60.0;

    x += cos(angle) * velocity * (update_delay / 1000.0);
    y += sin(angle) * velocity * (update_delay / 1000.0);
    pros::c::screen_print(TEXT_MEDIUM, 6, "X, Y -> %f, %f", x, y);

    //cout << *tstamp << " -> " << left_rev << ", " << right_rev << " -> " << 360 * (angle / (2.0 * M_PI)) << ", " << x << ", " << y << "\n";
    delete tstamp;
}

