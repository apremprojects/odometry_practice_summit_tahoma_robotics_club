#include "hal.h"
#include <string>
void HAL::toggle_left_wing(const bool b){
    Logger::getDefault()->log("left_wing " + std::to_string(b), DEPLOY_UPDATE);
    left_wing_status = b;
    left_wing.set_value(b);
}
void HAL::toggle_right_wing(const bool b){
    Logger::getDefault()->log("right_wing " + std::to_string(b), DEPLOY_UPDATE);
    right_wing_status = b;
    right_wing.set_value(b);
}
void HAL::intake_start(const bool direction){
    Logger::getDefault()->log("intake_direction " + std::to_string(direction), DEPLOY_UPDATE);
    isIntakingIntaking = direction;
    if(!direction){
        intake_motor.move_velocity(200);
    } else{
        intake_motor.move_velocity(-200);
    }
}
void HAL::intake_stop(){
    Logger::getDefault()->log("intake_stop", DEPLOY_UPDATE);
    intake_motor.move_velocity(0);
}