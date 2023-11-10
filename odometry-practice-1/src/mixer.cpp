#include "mixer.h"
void Mixer::update() {
    if(yaw > 127){
        yaw = 127;
    }
    else if(yaw < -127){
        yaw = -127;
    }
    if(throttle > 127){
        throttle = 127;
    }
    else if(throttle < -127){
        throttle = -127;
    }
    double max_rpm = 200;
    double left_target_rpm = (throttle / 127.0) * max_rpm + yaw;
    double right_target_rpm = (throttle / 127.0) * max_rpm - yaw;
    if(left_target_rpm > 200){
        left_target_rpm = 200 + yaw;
    }
    else if(left_target_rpm < -200){
        left_target_rpm = -200 + yaw;
    }
    if(right_target_rpm > 200){
        right_target_rpm = 200 - yaw;
    }
    else if(right_target_rpm < -200){
        right_target_rpm - -200 - yaw;
    }
    if(left_target_rpm >= 0){
        left.set_reversed(false);
        left.move_velocity(left_target_rpm);
    }
    else{
        left.set_reversed(true);
        left.move_velocity(abs(left_target_rpm));
    }
    if(right_target_rpm >= 0){
        right.set_reversed(true);
        right.move_velocity(right_target_rpm);
    }
    else{
        right.set_reversed(false);
        right.move_velocity(abs(right_target_rpm));
    }
}