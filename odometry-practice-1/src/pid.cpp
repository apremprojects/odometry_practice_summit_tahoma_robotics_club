#include "pid.h"
#include <iostream>

void PID::update(){
    double old_angle_error = target_angle - state->getAngle();
    double new_angle_error = target_angle - state->getAngle();
    double yaw_output = 0, throttle_output = 0;
    double p_a = 0, i_a = 0, d_a = 0;
    double v_rpm_target = 0;
    while(true) {
        if(running){
            //INTERIM SOLUTION REPLACING ROTATIONS SINCE START WITH HEADING
            new_angle_error = target_angle - state->getAngle();
            p_a = new_angle_error * p_g;
            i_a += (new_angle_error * (1.0 / update_freq)) * i_g;
            d_a = ((new_angle_error - old_angle_error) / (1.0 / update_freq)) * d_g;
            yaw_output = p_a + i_a + d_a;
            v_rpm_target = (target_velocity * 60.0) / (2.0 * M_PI * state->wheel_radius);
            throttle_output = (v_rpm_target / mixer->max_throttle_rpm) * 127.0;
            mixer->setYaw(-yaw_output);
            mixer->setThrottle(throttle_output);
            mixer->update();
            old_angle_error = target_angle - state->getAngle();
        }
        else{
            i_a = 0;
        }
        delay(1000 / update_freq);
    }
}