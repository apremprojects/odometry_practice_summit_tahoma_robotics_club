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
class State{
	public:
		State(const int left_port, const int right_port, const int imu_port, const int _update_freq);
		void update();
		double x = 0;
		double y = 0;
		double velocity = 0;
		double angle = 0;
		Motor left;
		Motor right;
		pros::Imu imu;
	private:
		int update_freq;
};