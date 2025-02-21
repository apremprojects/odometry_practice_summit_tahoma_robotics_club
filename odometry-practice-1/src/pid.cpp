#include "pid.h"
#include <iostream>

void PID::update(){
    double old_angle = state->getAngle();
    double new_angle = state->getAngle();
    double old_angle_error = target_angle - old_angle;
    double new_angle_error = target_angle - new_angle;
    double yaw_output = 0, throttle_output = 0;
    double p_a = 0, i_a = 0, d_a = 0;
    double v_rpm_target = 0;
    while(true) {
        if(running){
            //INTERIM SOLUTION REPLACING ROTATIONS SINCE START WITH HEADING
            new_angle_error = target_angle - state->getAngle();
            new_angle = state->getAngle();
            p_a = new_angle_error * p_g;
            i_a += (new_angle_error * (1.0 / update_freq)) * i_g;
            if(target_angle > new_angle){ //robot must turn counterclockwise
                d_a = ((old_angle - new_angle) / (1.0 / update_freq)) * d_g;
            }
            else { //robot must turn clockwise
                d_a = ((new_angle - old_angle) / (1.0 / update_freq)) * d_g;
            }

            if(p_a > 0 && d_a > 0){
                d_a = 0;
            }
            else if (p_a < 0 && d_a < 0){
                d_a = 0;
            }
            //d_a should be negative if approaching the target_angle, positive if going away
            Logger::getDefault()->log("PID -> " + std::to_string(p_a) + ", " + std::to_string(i_a) + ", " + std::to_string(d_a), DEBUG_MESSAGE);

            //d_a should be negative if approaching the target_angle, positive if going away
            yaw_output = p_a + i_a + d_a;
            yaw_output = std::clamp(yaw_output, -60.0, 60.0);
            v_rpm_target = (target_velocity * 60.0) / (2.0 * M_PI * state->wheel_radius);
            throttle_output = (v_rpm_target / mixer->max_throttle_rpm) * 127.0;
            mixer->setYaw(-yaw_output);
            mixer->setThrottle(throttle_output);
            mixer->update();
            old_angle_error = target_angle - state->getAngle();
            old_angle = state->getAngle();
        }
        else{
            i_a = 0;
        }
        delay(1000 / update_freq);
    }
}