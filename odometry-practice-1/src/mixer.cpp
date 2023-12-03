#include "mixer.h"
#include <algorithm>
void Mixer::update() {
    mutex.take(TIMEOUT_MAX);
    std::clamp(yaw, -127, 127);
    std::clamp(throttle, -127, 127);
    double yaw_rpm = (yaw / 127.0) * max_yaw_rpm;
    double throttle_rpm = max_throttle_rpm * (throttle / 127.0);
    double left_target_rpm = throttle_rpm + yaw_rpm;
    double right_target_rpm = throttle_rpm - yaw_rpm;
    mutex.give();
    //std::cout << left_target_rpm << ", " << right_target_rpm << "\n";
    if(left_target_rpm <= 0){
        left_one.set_reversed(false);
        left_two.set_reversed(false);
        left_three.set_reversed(false);
        left_one.move_velocity(-left_target_rpm);
        left_two.move_velocity(-left_target_rpm);
        left_three.move_velocity(-left_target_rpm);
    }
    else{
        left_one.set_reversed(true);
        left_two.set_reversed(true);
        left_three.set_reversed(true);
        left_one.move_velocity(left_target_rpm);
        left_two.move_velocity(left_target_rpm);
        left_three.move_velocity(left_target_rpm);
    }
    if(right_target_rpm <= 0){
        right_one.set_reversed(true);
        right_two.set_reversed(true);
        right_three.set_reversed(true);
        right_one.move_velocity(-right_target_rpm);
        right_two.move_velocity(-right_target_rpm);
        right_three.move_velocity(-right_target_rpm);
    }
    else{
        right_one.set_reversed(false);
        right_two.set_reversed(false);
        right_three.set_reversed(false);
        right_one.move_velocity(right_target_rpm);
        right_two.move_velocity(right_target_rpm);
        right_three.move_velocity(right_target_rpm);
    }
}