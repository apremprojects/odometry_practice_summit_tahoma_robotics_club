#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "mixer.h"
#include "state.h"
#include <mutex>
#include <string>
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
			std::lock_guard lock(mutex);
			target_angle = new_target_angle;
		}
		void setTargetVelocity(const double new_target_velocity){
			std::lock_guard lock(mutex);
			target_velocity = new_target_velocity;
		}
		double getTargetAngle(){
			std::lock_guard lock(mutex);
			return target_angle;
		}
		double getTargetVelocity(){
			std::lock_guard lock(mutex);
			return target_velocity;
		}
		void changeRunningState(const bool new_state){
			std::lock_guard lock(mutex);
			if(running!=new_state){
				mixer->setBrakeMode(new_state);
				running = new_state;
				Logger::getDefault()->log(std::to_string(running), PID_STATUS);
			}
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