#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
 using namespace pros;
 using namespace pros::literals;
 class Mixer{
	public:
		Mixer(const int left_port, const int right_port): left(left_port), right(right_port, true){}
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
		Mutex mutex;
		Motor left;
		Motor right;
};