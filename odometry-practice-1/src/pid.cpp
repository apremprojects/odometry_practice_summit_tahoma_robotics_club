#include "pid.h"

void PID::update(){
    double old_angle_error = target_angle - state->getAngle();
    double old_velocity_error = (target_velocity / 1000.0) - (state->getVelocity() / 1000.0);
    double new_angle_error = target_angle - state->getAngle();
    double new_velocity_error = (target_velocity / 1000.0) - (state->getVelocity() / 1000.0);
    double yaw_output = 0, throttle_output = 0;
    double p_a, i_a, d_a = 0;
    double p_v, i_v, d_v;
    while(true) {
        state->update();
        new_angle_error = target_angle - state->getAngle();
        new_velocity_error = (target_velocity / 1000.0) - (state->getVelocity() / 1000.0);
        p_a = new_angle_error * p_g;
        i_a += (new_angle_error * (1.0 / update_freq)) * i_g;
        d_a = ((new_angle_error - old_angle_error) / (1.0 / update_freq)) * d_g;
        p_v = new_velocity_error * p_g;
        i_v += (new_velocity_error * (1.0 / update_freq)) * i_g;
        d_v = ((new_velocity_error - old_velocity_error) / (1.0 / update_freq)) * d_g;
        yaw_output = p_a + i_a + d_a;
        throttle_output = p_v + i_v + d_v;
        mixer->setYaw(yaw_output);
        mixer->setThrottle(throttle_output);
        mixer->update();
        state->update();
        old_angle_error = target_angle - state->getAngle();
        old_velocity_error = (target_velocity / 1000.0) - (state->getVelocity() / 1000.0);
        delay(1000 / update_freq);
    }
}