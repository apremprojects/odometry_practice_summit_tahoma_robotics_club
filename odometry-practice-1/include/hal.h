#pragma once
#include "api.h"
#include "logger.h"
#include <mutex>
#include <cmath>
#include <string>
#include <queue>
using namespace pros;
using namespace pros::literals;
class HAL{ //hardware abstraction layer
	public:
		HAL(const int _left_wing_triwire_port, const int _right_wing_triwire_port, const int _intake_motor_port, const int _left_one_port, const int _left_two_port, const int _left_three_port, const int _right_one_port, const int _right_two_port, const int _right_three_port, const int _imu_port): left_wing(_left_wing_triwire_port), right_wing(_right_wing_triwire_port), intake_motor(_intake_motor_port), left_one(_left_one_port), left_two(_left_two_port), left_three(_left_three_port), right_one(_right_one_port), right_two(_right_two_port), right_three(_right_three_port), imu(_imu_port) {
			Logger::getDefault()->log("HAL::HAL()", FUNCTION_CALL);
		}
		void toggle_left_wing(const bool b);
		void toggle_right_wing(const bool b);
		void intake_start(const bool direction);
		void intake_stop();
		Motor& get_left_one(){
			return left_one;
		}
		Motor& get_left_two(){
			return left_two;
		}
		Motor& get_left_three(){
			return left_three;
		}
		Motor& get_right_one(){
			return right_one;
		}
		Motor& get_right_two(){
			return right_two;
		}
		Motor& get_right_three(){
			return right_three;
		}
		Imu& get_imu(){
			return imu;
		}
	private:
		Motor intake_motor;
		Motor left_one;
		Motor left_two;
		Motor left_three;
		Motor right_one;
		Motor right_two;
		Motor right_three;
		Imu imu;
		bool left_wing_status;
		bool right_wing_status;
		bool isIntakingIntaking;
		ADIDigitalOut left_wing;
		ADIDigitalOut right_wing;
};