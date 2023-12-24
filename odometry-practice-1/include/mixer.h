#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#include "api.h"
#include "logger.h"
#include "hal.h"
 using namespace pros;
 using namespace pros::literals;
 class Mixer{
	public:
		Mixer(HAL *_hal): hal(_hal){
			Logger *logger = Logger::getDefault();
			hal->get_left_one().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			hal->get_left_two().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			hal->get_left_three().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			hal->get_right_one().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			hal->get_right_two().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			hal->get_right_three().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			logger->log("Mixer::Mixer() -> mixer.h", FUNCTION_CALL);
		}
		void update();
		void setThrottle(const int _throttle){
			mutex.take(TIMEOUT_MAX);
			throttle = _throttle;
			mutex.give();
		}
		void setYaw(const int _yaw){
			mutex.take(TIMEOUT_MAX);
			yaw = _yaw;
			mutex.give();
		}
		double getThrottle(){
			mutex.take(TIMEOUT_MAX);
			double _throttle = throttle;
			mutex.give();
			return _throttle;
		}
		double getYaw(){
			mutex.take(TIMEOUT_MAX);
			double _yaw = yaw;
			mutex.give();
			return _yaw;
		}
		void set_control_point(const bool b){
			mutex.take(TIMEOUT_MAX);
			double isForward = b;
			mutex.give();
		}
	private:
		int yaw = 0; //-127 -> 127
		int throttle = 0; //-127 -> 127
		double max_throttle_rpm = 200;
		double max_yaw_rpm = 200;
		bool isForward = true;
		Mutex mutex;
		HAL *hal;
};