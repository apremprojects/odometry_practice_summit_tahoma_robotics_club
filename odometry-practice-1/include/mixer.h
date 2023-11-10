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
		int yaw = 0; //-127 -> 127
		int throttle = 0; //-127 -> 127
		Mixer(const int left_port, const int right_port): left(left_port), right(right_port, true){}
		void update();
	private:
		Motor left;
		Motor right;
};