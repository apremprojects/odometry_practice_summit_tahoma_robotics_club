#pragma once
#define _USE_MATH_DEFINES
#include "main.h"
#include "mixer.h"
#include "logger.h"
#include "hal.h"
#include "warnings.h"
#include "state.h"
#include "pid.h"
#include <iostream>
#include <cmath>
#include <string>
#include <list>
#include <utility>
#include <csignal>
#include "Eigen/Dense"
using namespace pros;

struct MotionCmd{
	MotionCmd(const double _max_velocity, const double _a_time, const double _end_x, const double _end_y, const bool _decelerate): max_velocity(_max_velocity), a_time(_a_time), end_x(_end_x), end_y(_end_y), decelerate(_decelerate){}
	double max_velocity;
	double a_time;
	double end_x;
	double end_y;
	bool decelerate;
};

struct Status{
	Status(const bool b): done(b){}
	bool done = false;
};

class Robot {
	public:
		Robot(const double start_angle, const double start_x, const double start_y, const int _update_freq) : update_freq(_update_freq){
			delay(3000);
			logger = Logger::getDefault();
			logger->log("Robot::Robot()", FUNCTION_CALL);
			//wait for PROS kernel to do all its init stuff, it isn't ready when global stuff is setup
			hal = new HAL(1, 5, 4, 1, 2, 3, 8, 9, 10, 17);
			mixer = new Mixer(hal);
			state = new State(hal, mixer, start_x, start_y, start_angle, update_freq);
			pid = new PID(80, 0, 10, update_freq, mixer, state); //40, 0,
			//~3.14 first peak ampltiude attempting first angle set to 1.57 with 80, 0, - 10 -> 1.57 overshoot
			//~1.58 fior peak amp with 80, 0, 10 - 0.01 overshoot
			//60, 0, 10 - 2/16/2025 - notes too much oscillation, increase d term
			//80, 0, 15 - too even more oscillation than last terms, maybe too much d?
			//t=5.8 -> pos d term cuz moving away from setpoint, neg d term cuz moving towards set point
			//80, 0, 5 - still oscillation but not as bad?
			pid_task = new Task([this](){pid->update();});
			state_task = new Task([this](){
				while(true){
					uint32_t st = millis();
					state->update();
					uint32_t et = millis();
					delay(1000 / update_freq - (et-st));
				}
			});
			goto_task = new Task(goto_func);
		}
		~Robot(){
			goto_task->remove();
			pid_task->remove();
			state_task->remove();
			delete state;
			delete mixer;
			delete pid;
			delete goto_task;
			delete pid_task;
			delete state_task;
		}
		void set_angle(const double new_angle) {
			state->setAngle(new_angle);
		}
		void set_angle(double new_angle, const bool blocking){
			logger->log(std::to_string(new_angle), TARGET_ANGLE_UPDATE);
			pid->changeRunningState(true);
			pid->setTargetAngle(new_angle);
			if(blocking){
				//wait until the robot reaches the target angle, check every 20ms
				double o_angle = get_angle();
				while(fmod(abs(new_angle - get_angle()), 2 * M_PI) > 0.1 || abs(get_angle() - o_angle) > 0.1){
					o_angle = get_angle();
					delay(20);
					//logger->log(std::to_string(fmod(abs(new_angle - get_angle()), 2 * M_PI)), DEBUG_MESSAGE);
				}
			}
			return;
		}
		void set_velocity(const double new_velocity, const bool blocking){
			logger->log(std::to_string(new_velocity), TARGET_VELOCITY_UPDATE);
			//enable pid
			pid->changeRunningState(true);
			pid->setTargetVelocity(new_velocity);
			if(blocking){
				while(abs(new_velocity - get_velocity()) < 10.0){
					delay(20);
				}
			}
		}
		void set_yaw(const int new_yaw){
			//disable pid
			logger->log(std::to_string(new_yaw), YAW_UPDATE);
			pid->changeRunningState(false);
			mixer->setYaw(new_yaw);
			mixer->update();
		}
		void set_throttle(const int new_throttle){
			//disable pid
			logger->log(std::to_string(new_throttle), THROTTLE_UPDATE);
			pid->changeRunningState(false);
			mixer->setThrottle(new_throttle);
			mixer->update();
		}
		double get_angle(){
			return state->getAngle();
			//return 0;
		}
		double get_velocity(){
			return state->getVelocity();
			//return 0;
		}
		double getLeftRPM(){
			return state->getLeftRPM();
		}
		double getRightRPM(){
			return state->getRightRPM();
		}
		double getX(){
			return state->getX();
		}
		double getY(){
			return state->getY();
		}
		void setPos(const double x, const double y){
			state->setPos(x,y);
		}
		Status* goto_pos(const double max_velocity, const double a_time, const double end_x, const double end_y, const bool decelerate){
			MotionCmd cmd(max_velocity, a_time, end_x, end_y, decelerate);
			Status *status = new Status(false);
			{
				std::lock_guard<Mutex> lock(mutex);
				queue.push(cmd);
				status_queue.push_back(status);
			}
			goto_task->notify();
			//return status_queue.back();
			return status;
		}
		void acknowledge(){
			std::lock_guard<Mutex> lock(mutex);
			delete status_queue.front();
			status_queue.pop_front();
		}
		double get_angle_between(const double cur_x, const double cur_y, const double end_x, const double end_y, const double angle){
			const double target_heading = atan2(end_y - cur_y, end_x - cur_x);
			//const double rotations = (angle - target_heading) / (2.0 * M_PI); // turns left
			return round_heading_to_rotations(target_heading, angle);

			//return round(rotations) * (2.0 * M_PI) + target_heading;
			//return target_heading;
		}

		double round_heading_to_rotations(const double target_heading, const double angle){
			double lower = 0;
			double upper = 0;
			lower = std::floor((angle) / (2.0 * M_PI)) * (2.0 * M_PI) + target_heading;
			upper = std::ceil((angle) / (2.0 * M_PI)) * (2.0 * M_PI) + target_heading;
			double res = 0;
			if(fabs(angle - lower) < fabs(angle - upper)){
				res = lower;
			}
			else{
				res = upper;
			}
			logger->log("RHTR " + std::to_string(lower) + ", " + std::to_string(upper) + ", " + std::to_string(res), DEBUG_MESSAGE);
			return res;
		}


		double get_distance(const double cur_x, const double cur_y, const double end_x, const double end_y){
			return sqrt(pow(end_x - cur_x, 2) + pow(end_y - cur_y, 2));
		}
		bool reached(const double cur_x, const double cur_y, const double end_x, const double end_y){
			return abs(cur_x - end_x) < 10 && abs(cur_y - end_y) < 10;
		}
		void set_control_point(const bool b){
			logger->log(std::to_string(b), CONTROL_POINT);
			if(b){
				Warning::getDefault()->raise(0);
			}
			else{
				Warning::getDefault()->revoke(0);
			}
			if(!b){
				Warning::getDefault()->raise(1);
			}
			else{
				Warning::getDefault()->revoke(1);
			}
			{
				std::lock_guard<Mutex> lock(mutex);
				isForward = b;
				mixer->set_control_point(b);
				state->set_control_point(b);
			}
		}
		HAL* get_hal(){
			return hal;
		}
		bool get_control_point(){
			return isForward;
		}
		std::list<Status*> status_queue;
		State *state; // = new State(7, 1, 2, update_freq * 2);
	private:
		double update_freq = -1;
		bool isForward = true;
		Mixer *mixer; // = new Mixer(7, 1);
		PID *pid; // = new PID(140, 0, 1, update_freq, mixer, state)
		HAL *hal;
		std::queue<MotionCmd> queue;
		Task *goto_task;
		Task *pid_task;
		Task *state_task;
		Logger *logger;
		Mutex mutex;
		double threshold = 50;
		double endgame_dist = 200;
		std::function<void()> goto_func = [this](){
			while(pros::Task::notify_take(true, TIMEOUT_MAX)){ //timeout
				{
					std::lock_guard<Mutex> lock(mutex);
					while(!queue.empty()){
						MotionCmd cmd = queue.front();
						logger->log(std::to_string(cmd.end_x) + " " + std::to_string(cmd.end_y) + " " + std::to_string(cmd.max_velocity) + " " + std::to_string(cmd.a_time), TARGET_POS_UPDATE);
						queue.pop();
						mutex.give();
						//execute motioncmd somehow
						double cur_x = state->getX();
						double cur_y = state->getY();
						double start_x = cur_x;
						double start_y = cur_y;
						//Eigen::Vector2d target_vector{cmd.end_x - start_x, cmd.end_y - start_y};
						double p = 0, i = 0, d = 0;
						double p_g = 0.01, i_g = 0, d_g = 0;
						double cur_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						double old_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						double cur_angle = get_angle_between(cur_x, cur_y, cmd.end_x, cmd.end_y, get_angle());
						double old_angle = get_angle_between(cur_x, cur_y, cmd.end_x, cmd.end_y, get_angle());
						double endgame_mode = false;
						double endgame_angle = 0;
						bool reached = false;
						set_velocity(0, false);
						set_angle(cur_angle, true);
						while(!reached){
							cur_x = state->getX();
							cur_y = state->getY();
							cur_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
							cur_angle = get_angle_between(cur_x, cur_y, cmd.end_x, cmd.end_y, get_angle());

							p = p_g * cur_dist; //proportioning error
							i += i_g * cur_dist; //integrating error
							d = d_g * cur_dist;//derivative of error
							//if angle error > 0.1 rads ~6 deg stop and try rotating

							/*
							double lateral_error = 0;
							if(target_vector.norm() != 0){
								Eigen::Vector2d actual_vector = {cur_x - start_x, cur_y - start_y};
								lateral_error = (actual_vector - ((actual_vector.dot(target_vector) / target_vector.dot(target_vector)) * target_vector)).norm();
								logger->log("LAT_ERROR " + std::to_string(lateral_error), DEBUG_MESSAGE);
							}
							*/

							if((fabs(get_angle() - cur_angle) < 0.1 && (fabs(cur_angle - old_angle) * 50) < 0.1) && cur_dist > endgame_dist){
								set_angle(cur_angle, false);
								if(cmd.decelerate){
									set_velocity(std::min((p + i + d) * cmd.max_velocity, cmd.max_velocity), false);
								}
								else{
									set_velocity(cmd.max_velocity, false);
								}
							}
							else if (cur_dist <= endgame_dist){
								//blocking lock to endgame angle
								//merely go forwards
								if(!endgame_mode){ //if not endgame mode
									endgame_angle = cur_angle; //set endgame_angle to current angle between robot and target
									endgame_mode = true; //enable endgame mode
									set_velocity(0, false);
									set_angle(endgame_angle, true);
									logger->log("ENDGAME MODE", DEBUG_MESSAGE);
								}
								if(cmd.decelerate){
									set_velocity(std::min((p + i + d) * cmd.max_velocity, cmd.max_velocity), false);
								}
								else{
									set_velocity(cmd.max_velocity, false);
								}
								
							}
							else{
								set_angle(cur_angle, false);
								set_velocity(0, false);
							}

							if(cur_dist <= threshold){
								/*if(cur_dist >= old_dist){
									reached = true;
								}*/
								reached = true;
							}
							old_dist = cur_dist;
							old_angle = cur_angle;
							delay(20);
						}
						if(cmd.decelerate){
							set_velocity(0, false);
						}
						status_queue.front()->done = true;
					}
				}
			}
		};
};