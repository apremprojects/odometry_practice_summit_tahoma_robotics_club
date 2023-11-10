#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "mixer.h"
#include "state.h"
/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
 using namespace pros;
 using namespace pros::literals;
class PID{
	public:
		PID(double _p_g, double _i_g, double _d_g, int _update_freq, Mixer *_mixer, State *_state): p_g(_p_g), i_g(_i_g), d_g(_d_g), update_freq(_update_freq), mixer(_mixer), state(_state){}
		void update();
		double p_g, i_g, d_g;
		double target_angle = 0;
		double target_velocity = 0;
		int update_freq; //hz
		Mixer *mixer;
		State *state;
};