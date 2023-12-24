#include "mixer.h"
#include "logger.h"
#include <algorithm>
void Mixer::update() {
    mutex.take(TIMEOUT_MAX);
    yaw = std::clamp(yaw, -127, 127);
    throttle = std::clamp(throttle, -127, 127);
    double yaw_rpm = (yaw / 127.0) * max_yaw_rpm;
    double throttle_rpm = max_throttle_rpm * (throttle / 127.0);
    if(!isForward){
        throttle_rpm = -throttle_rpm;
        yaw_rpm = -yaw_rpm;
    }
    double left_target_rpm = throttle_rpm + yaw_rpm;
    double right_target_rpm = throttle_rpm - yaw_rpm;
    mutex.give();
    hal->get_left_one().move_velocity(left_target_rpm);
    hal->get_left_two().move_velocity(left_target_rpm);
    hal->get_left_three().move_velocity(left_target_rpm);
    hal->get_right_one().move_velocity(right_target_rpm);
    hal->get_right_two().move_velocity(right_target_rpm);
    hal->get_right_three().move_velocity(right_target_rpm);
    //if left rpm or right rpm > 200, then find the max rpm (std::max(left_rpm. right_rpm)), then multiply left_rpm and right_rpm by 200/max_rpm
}