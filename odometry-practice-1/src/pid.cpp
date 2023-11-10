#include "pid.h"

void PID::update(){
    double old_angle_error = target_angle - state->angle;
    double old_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);
    double new_angle_error = target_angle - state->angle;
    double new_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);
    double yaw_output = 0, throttle_output = 0;
    double p_a, i_a, d_a = 0;
    double p_v, i_v, d_v;
    while(true){
        pros::c::screen_print(TEXT_MEDIUM, 7, "p_g, i_g, d_g -> %f, %f, %f", p_g, i_g, d_g);
        state->update();
        pros::c::screen_print(TEXT_MEDIUM, 8, "Target Velocity -> %f", target_velocity);
        pros::c::screen_print(TEXT_MEDIUM, 9, "Actual Velocity -> %f", state->velocity);
        new_angle_error = target_angle - state->angle;

        new_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);

        p_a = new_angle_error * p_g;
        i_a += (new_angle_error * (1.0 / update_freq)) * i_g;
        d_a = ((new_angle_error - old_angle_error) / (1.0 / update_freq)) * d_g;
        pros::c::screen_print(TEXT_MEDIUM, 2, "p_a, i_a, d_a -> %f, %f, %f", p_a, i_a, d_a);

        p_v = new_velocity_error * p_g;
        i_v += (new_velocity_error * (1.0 / update_freq)) * i_g;
        d_v = ((new_velocity_error - old_velocity_error) / (1.0 / update_freq)) * d_g;
        pros::c::screen_print(TEXT_MEDIUM, 3, "p_v, i_v, d_v -> %f, %f, %f", p_v, i_v, d_v);

        yaw_output = p_a + i_a + d_a;
        pros::c::screen_print(TEXT_MEDIUM, 4, "PID yaw_output -> %f", yaw_output);

        throttle_output = p_v + i_v + d_v;
        pros::c::screen_print(TEXT_MEDIUM, 5, "PID throttle_output -> %f", throttle_output);

        //cout << "PID Yaw -> " << yaw_output << "\n";
        //cout << "PID Throttle ->" << throttle_output << "\n";

        mixer->yaw = yaw_output;
        mixer->throttle = throttle_output;
        mixer->update();
        state->update();
        old_angle_error = target_angle - state->angle;
        old_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);
        delay(1000 / update_freq);
    }
}