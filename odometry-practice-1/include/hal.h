#pragma once
#include "api.h"
#include "logger.h"
#include <cmath>
#include <string>
#include <queue>
using namespace pros;
using namespace pros::literals;
class HAL{ //hardware abstraction layer
	public:
		HAL(const int _left_wing_triwire_port, const int _right_wing_triwire_port, const int _intake_motor_port, const int _left_one_port, const int _left_two_port, const int _left_three_port, const int _right_one_port, const int _right_two_port, const int _right_three_port, const int _imu_port): left_wing(_left_wing_triwire_port, false), right_wing(_right_wing_triwire_port, false), intake_motor(_intake_motor_port), left_one(_left_one_port, E_MOTOR_GEAR_BLUE, true), left_two(_left_two_port, E_MOTOR_GEAR_BLUE, true), left_three(_left_three_port, E_MOTOR_GEAR_BLUE, true), right_one(_right_one_port, E_MOTOR_GEAR_BLUE), right_two(_right_two_port, E_MOTOR_GEAR_BLUE), right_three(_right_three_port, E_MOTOR_GEAR_BLUE), imu(_imu_port) {
			Logger::getDefault()->log("HAL::HAL()", FUNCTION_CALL);
			left_wing.set_value(false);
			right_wing.set_value(false);
			uint32_t *tstamp = new uint32_t(millis());
			start_left_ticks = left_one.get_raw_position(tstamp);
			start_right_ticks = right_one.get_raw_position(tstamp);
			delete tstamp;
		}
		void toggle_left_wing(const bool b);
		void toggle_right_wing(const bool b);
		void intake_start(const bool direction);
		void intake_stop();
		void set_left_velocity(double rpm){
			rpm /= mw_ratio;
			left_one.move_velocity(rpm);
			left_two.move_velocity(rpm);
			left_three.move_velocity(rpm);
		}
		double get_left_velocity(){
			return left_one.get_actual_velocity() * mw_ratio;
		}
		void set_right_velocity(double rpm){
			rpm /= mw_ratio;
			right_one.move_velocity(rpm);
			right_two.move_velocity(rpm);
			right_three.move_velocity(rpm);
		}
		double get_right_velocity(){
			return right_one.get_actual_velocity() * mw_ratio;
		}
		double get_left_rotations(){
			return (get_left_ticks() / 300.0) * mw_ratio;
		}
		double get_right_rotations(){
			return (get_right_ticks() / 300.0) * mw_ratio;
		}
		int32_t get_left_ticks(){
			uint32_t *tstamp = new uint32_t(millis());
			int32_t tmp = left_one.get_raw_position(tstamp) - start_left_ticks;
			delete tstamp;
			return tmp;
		}
		int32_t get_right_ticks(){
			//500 ticks = 1 wheel rotation
			//900 ticks = 1 motor rotation
			uint32_t *tstamp = new uint32_t(millis());
			int32_t tmp = right_one.get_raw_position(tstamp) - start_right_ticks;
			delete tstamp;
			return tmp;
		}
		void set_brake_mode(const pros::motor_brake_mode_e mode){
			left_one.set_brake_mode(mode);
			left_two.set_brake_mode(mode);
			left_three.set_brake_mode(mode);
			right_one.set_brake_mode(mode);
			right_two.set_brake_mode(mode);
			right_three.set_brake_mode(mode);
		}
		pros::Imu& get_imu(){
			return imu;
		}
		std::array<double,7> get_temperatures(){
			std::array<double,7> r;
			r[0] = left_one.get_temperature();
			r[1] = left_two.get_temperature();
			r[2] = left_three.get_temperature();
			r[3] = right_one.get_temperature();
			r[4] = right_two.get_temperature();
			r[5] = right_three.get_temperature();
			r[6] = intake_motor.get_temperature();
			return r;
		}
		std::array<double,7> get_rpms(){
			std::array<double,7> r;
			r[0] = left_one.get_actual_velocity();
			r[1] = left_two.get_actual_velocity();
			r[2] = left_three.get_actual_velocity();
			r[3] = right_one.get_actual_velocity();
			r[4] = right_two.get_actual_velocity();
			r[5] = right_three.get_actual_velocity();
			r[6] = intake_motor.get_actual_velocity();
			return r;
		}
		std::array<double,7> get_torques(){
			std::array<double,7> r;
			r[0] = left_one.get_torque();
			r[1] = left_two.get_torque();
			r[2] = left_three.get_torque();
			r[3] = right_one.get_torque();
			r[4] = right_two.get_torque();
			r[5] = right_three.get_torque();
			r[6] = intake_motor.get_torque();
			return r;
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
		const double mw_ratio = 3.0 / 5.0; //motor wheel gear ratio
		//const double mw_ratio = 9.0 / 5.0;
		//5 motor rotation 3 wheel rotations
		bool left_wing_status;
		bool right_wing_status;
		bool isIntakingIntaking;
		int32_t start_left_ticks = 0, start_right_ticks = 0;
		ADIDigitalOut left_wing;
		ADIDigitalOut right_wing;
};