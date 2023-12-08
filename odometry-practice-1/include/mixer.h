#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "logger.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
 using namespace pros;
 using namespace pros::literals;
 class Mixer{
	public:
		Mixer(const int left_one_port, const int left_two_port, const int left_three_port, const int right_one_port, const int right_two_port, const int right_three_port): left_one(left_one_port), left_two(left_two_port), left_three(left_three_port), right_one(right_one_port), right_two(right_two_port), right_three(right_three_port){
			Logger *logger = Logger::getDefault();
			left_one.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			left_two.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			left_three.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			right_one.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			right_two.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			right_three.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			logger->log("Mixer::Mixer() -> mixer.h");
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
	private:
		int yaw = 0; //-127 -> 127
		int throttle = 0; //-127 -> 127
		double max_throttle_rpm = 200;
		double max_yaw_rpm = 200;
		Mutex mutex;
		Motor left_one;
		Motor left_two;
		Motor left_three;
		Motor right_one;
		Motor right_two;
		Motor right_three;
};