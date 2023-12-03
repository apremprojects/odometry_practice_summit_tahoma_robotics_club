#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#define _USE_MATH_DEFINES

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
#include <mutex>
#include <cmath>
 using namespace pros;
 using namespace pros::literals;
class State{
	public:
		State(const int left_one_port, const int left_two_port, const int left_three_port, const int right_one_port, const int right_two_port, const int right_three_port, const int _imu_port, const int _update_freq);
		State(const int left_one_port, const int left_two_port, const int left_three_port, const int right_one_port, const int right_two_port, const int right_three_port, const int _imu_port, const double start_x, const double start_y, const double start_angle, const int _update_freq);
		void update();
		double getX(){
			mutex.take(TIMEOUT_MAX);
			double _x = x;
			mutex.give();
			return _x;
		}
		void setX(const double new_x){
			mutex.take(TIMEOUT_MAX);
			x = new_x;
			mutex.give();
		}
		double getY(){
			mutex.take(TIMEOUT_MAX);
			double _y = y;
			mutex.give();
			return _y;
		}
		void setY(const double new_y){
			mutex.take(TIMEOUT_MAX);
			y = new_y;
			mutex.give();
		}
		double getVelocity(){
			mutex.take(TIMEOUT_MAX);
			double _velocity = velocity;
			mutex.give();
			return _velocity;
		}
		double getAngle(){
			mutex.take(TIMEOUT_MAX);
			double _angle = angle;
			mutex.give();
			return _angle;
		}
		void setAngle(const double new_angle){
			mutex.take(TIMEOUT_MAX);
			angle = new_angle;
			mutex.give();
		}
		void initializeImu();
	private:
		double x = 0;
		double y = 0;
		double velocity = 0;
		double angle = 0;
		double start_angle = 0;
		int update_freq;
		int imu_port;
		Motor left_one;
		Motor left_two;
		Motor left_three;
		Motor right_one;
		Motor right_two;
		Motor right_three;
		pros::Imu imu;//issue explanation
		Mutex mutex;
};