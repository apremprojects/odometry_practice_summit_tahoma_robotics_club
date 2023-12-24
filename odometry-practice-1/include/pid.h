#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "mixer.h"
#include "state.h"
#include <mutex>
/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
 using namespace pros;
 using namespace pros::literals;
class PID{
	public:
		PID(double _p_g, double _i_g, double _d_g, int _update_freq, Mixer *_mixer, State *_state): p_g(_p_g), i_g(_i_g), d_g(_d_g), update_freq(_update_freq), mixer(_mixer), state(_state){
			Logger *logger = Logger::getDefault();
			logger->log("PID::PID() -> pid.h", FUNCTION_CALL);
		}
		void update();
		void setTargetAngle(const double new_target_angle){
			mutex.take(TIMEOUT_MAX);
			target_angle = new_target_angle;
			mutex.give();
		}
		void setTargetVelocity(const double new_target_velocity){
			mutex.take(TIMEOUT_MAX);
			target_velocity = new_target_velocity;
			mutex.give();
		}
		double getTargetAngle(){
			mutex.take(TIMEOUT_MAX);
			double _target_angle = target_angle;
			mutex.give();
			return _target_angle;
		}
		double getTargetVelocity(){
			mutex.take(TIMEOUT_MAX);
			double _target_velocity = target_velocity;
			mutex.give();
			return _target_velocity;
		}
		void changeRunningState(const bool new_state){
			mutex.take(TIMEOUT_MAX);
			running = new_state;
			mutex.give();
		}
	private:
		bool running = true;
		double p_g, i_g, d_g;
		double target_angle = 0;
		double target_velocity = 0;
		int update_freq; //hz
		Mutex mutex;
		Mixer *mixer;
		State *state;
};