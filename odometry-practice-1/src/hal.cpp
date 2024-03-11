#include "hal.h"
#include "warnings.h"
#include <string>
void HAL::toggle_left_wing(const bool b){
    Logger::getDefault()->log("left_wing " + std::to_string(b), DEPLOY_UPDATE);
    if(b){
        Warning::getDefault()->raise(2);
    }
    else{
        Warning::getDefault()->revoke(2);
    }
    left_wing_status = b;
    left_wing.set_value(b);
}
void HAL::toggle_right_wing(const bool b){
    Logger::getDefault()->log("right_wing " + std::to_string(b), DEPLOY_UPDATE);
    if(b){
        Warning::getDefault()->raise(3);
    }
    else{
        Warning::getDefault()->revoke(3);
    }
    right_wing_status = b;
    right_wing.set_value(b);
}
void HAL::intake_start(const bool direction){
    Logger::getDefault()->log("intake_direction " + std::to_string(direction), DEPLOY_UPDATE);
    //reversing motor directions due to emergency hardware change 1/20/2024
    if(direction){
        Warning::getDefault()->raise(4);
        //intake_motor.move_velocity(-600);
        intake_motor.move(-127);
    }
    else{
        Warning::getDefault()->raise(5);
        //intake_motor.move_velocity(600);
        intake_motor.move(127);
    }
    isIntakingIntaking = direction;
}
void HAL::intake_stop(){
    Warning::getDefault()->revoke(4);
    Warning::getDefault()->revoke(5);
    Logger::getDefault()->log("intake_stop", DEPLOY_UPDATE);
    intake_motor.move_velocity(0);
}