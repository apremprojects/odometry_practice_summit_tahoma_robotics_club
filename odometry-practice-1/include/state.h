#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
#include <mutex>
 using namespace pros;
 using namespace pros::literals;
class State{
	public:
		State(const int left_port, const int right_port, const int imu_port, const int _update_freq);
		void update();
		double getX(){
			mutex.take(TIMEOUT_MAX);
			double _x = x;
			mutex.give();
			return _x;
		}
		double getY(){
			mutex.take(TIMEOUT_MAX);
			double _y = y;
			mutex.give();
			return _y;
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
	private:
		double x = 0;
		double y = 0;
		double velocity = 0;
		double angle = 0;
		int update_freq;
		Motor left;
		Motor right;
		pros::Imu imu;
		Mutex mutex;
};