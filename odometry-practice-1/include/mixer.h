#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#include "api.h"
#include "logger.h"
#include "hal.h"
 using namespace pros;
 using namespace pros::literals;
 class Mixer{
	public:
		Mixer(HAL *_hal): hal(_hal){
			Logger *logger = Logger::getDefault();
			setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
			logger->log("Mixer::Mixer() -> mixer.h", FUNCTION_CALL);
		}
		void update();
		void setBrakeMode(const bool b){
			if(!b){
				hal->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
			else{
				hal->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			}
		}
		void setThrottle(const int _throttle){
			std::lock_guard lock(mutex);
			throttle = _throttle;
		}
		void setYaw(const int _yaw){
			std::lock_guard lock(mutex);
			yaw = _yaw;
		}
		double getThrottle(){
			std::lock_guard lock(mutex);
			return throttle;
		}
		double getYaw(){
			std::lock_guard lock(mutex);
			return yaw;
		}
		void set_control_point(const bool b){
			std::lock_guard lock(mutex);
			isForward = b;
		}
	private:
		int yaw = 0; //-127 -> 127
		int throttle = 0; //-127 -> 127
		double max_throttle_rpm = 200;
		double max_yaw_rpm = 200;
		bool isForward = true;
		Mutex mutex;
		HAL *hal;
};