#include "state.h"
#include "logger.h"

State::State(HAL *_hal, Mixer *_mixer, const double start_x, const double start_y, const double _start_angle, const int _update_freq): hal(_hal), mixer(_mixer), start_angle(_start_angle), update_freq(_update_freq) {
    Logger *logger = Logger::getDefault();
	logger->log("State::State() -> state.cpp", FUNCTION_CALL);
    std::lock_guard lock(mutex);
    //F.setIdentity();
    //I.setIdentity();
    //B.setZero();
    H << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0,
		 0, 0, 0, 1,
		 0, 0, 0, 1;
    R << 10.0, 0, 0, 0, 0,
		 0, 10.0, 0, 0, 0,
		 0, 0, 0, 0, 0,
		 0, 0, 0, 0.0001, 0,
		 0, 0, 0, 0, 0.0001;
    hal->get_imu().reset(true);
    //z << start_x, start_y, 0, start_angle-hal->get_imu().get_rotation(), start_angle-hal->get_imu().get_rotation();
    z(0) = start_x;
    z(1) = start_y;
    z(2) = 0;
    z(3) = start_angle;
    z(4) = start_angle;
    prev_state.first = {start_x, start_y, 0, start_angle-hal->get_imu().get_rotation()};
    prev_state.second = P;
    hal->get_imu().set_rotation(0);
}

void State::update(){
    //predict_state, predict_var = predict(prev_state)
    prev_state = predict(prev_state);
    //new_state, new_var = update(predict_state, predict_var, measurement)
    new_state = update(prev_state);
    //Logger::getDefault()->log("raw_angle_imu -> " + std::to_string(z(3)), DEBUG_MESSAGE);
    //Logger::getDefault()->log("raw_angle_odom -> " + std::to_string(z(4)), DEBUG_MESSAGE);
    //Logger::getDefault()->log("state -> " + toString(new_state.first), DEBUG_MESSAGE);
    Logger::getDefault()->log(std::to_string(new_state.first(0)) + ", " + std::to_string(new_state.first(1)), POS_UPDATE);
    Logger::getDefault()->log(std::to_string(new_state.first(3)), ANGLE_UPDATE);
    prev_state = new_state;
}