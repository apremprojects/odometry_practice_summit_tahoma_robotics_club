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
using namespace pros;

struct MotionCmd{
	MotionCmd(const double _max_velocity, const double _a_time, const double _end_x, const double _end_y): max_velocity(_max_velocity), a_time(_a_time), end_x(_end_x), end_y(_end_y){}
	double max_velocity;
	double a_time;
	double end_x;
	double end_y;
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
			hal = new HAL(2, 1, 5, 1, 2, 3, 8, 9, 10, 7);
			mixer = new Mixer(hal);
			state = new State(hal, mixer, start_x, start_y, start_angle, update_freq);
			pid = new PID(40, 0, 4, update_freq, mixer, state);
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
		void set_angle(const double new_angle, const bool blocking){
			logger->log(std::to_string(new_angle), TARGET_ANGLE_UPDATE);
			//enable pid
			pid->changeRunningState(true);
			pid->setTargetAngle(new_angle);
			double o_angle = get_angle();
			double n_angle = get_angle();
			if(blocking){
				//wait until the robot reaches the target angle, check every 20ms
				n_angle = get_angle();
				while(abs(n_angle - new_angle) > 0.1){
					delay(20);
				}
				o_angle = n_angle;
			}
			return;
		}
		void set_velocity(const double new_velocity, const bool blocking){
			logger->log(std::to_string(new_velocity), TARGET_VELOCITY_UPDATE);
			//enable pid
			pid->changeRunningState(true);
			pid->setTargetVelocity(new_velocity);
			if(blocking){
				//wait until the robot reaches the target velocity, check every 5ms
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
		Status* goto_pos(const double max_velocity, const double a_time, const double end_x, const double end_y, const bool blocking){
			MotionCmd cmd(max_velocity, a_time, end_x, end_y);
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
		double get_angle_between(const double cur_x, const double cur_y, const double end_x, const double end_y){
			return atan2(end_y - cur_y, end_x - cur_x);
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
	private:
		double update_freq = -1;
		bool isForward = true;
		State *state; // = new State(7, 1, 2, update_freq * 2);
		Mixer *mixer; // = new Mixer(7, 1);
		PID *pid; // = new PID(140, 0, 1, update_freq, mixer, state)
		HAL *hal;
		std::queue<MotionCmd> queue;
		Task *goto_task;
		Task *pid_task;
		Task *state_task;
		Logger *logger;
		Mutex mutex;
		double acc_dist = 10;
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
						double p = 0, i = 0, d = 0;
						double p_g = 10, i_g = 0.001, d_g = 0;
						double cur_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						double old_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						double angle = get_angle_between(cur_x, cur_y, cmd.end_x, cmd.end_y);
						double new_angle = get_angle_between(cur_x, cur_y, cmd.end_x, cmd.end_y);
						while(cur_dist <= old_dist || cur_dist > acc_dist){
							cur_x = state->getX();
							cur_y = state->getY();
							cur_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
							new_angle = get_angle_between(cur_x, cur_y, cmd.end_x, cmd.end_y);
							if(abs(new_angle - angle) < 1.0){
								angle = new_angle;
							}
							else{
								break;
							}
							set_angle(angle, true);
							p = p_g * cur_dist; //proportioning error
							i += i_g * cur_dist; //integrating error
							d = d_g * (cur_dist - old_dist);//derivative of error
							set_velocity(std::min(p + i + d, cmd.max_velocity), false);
							delay(20);
						}
						set_velocity(0, true);
						status_queue.front()->done = true;
						old_dist = cur_dist;
					}
				}
			}
		};
};

class GUI{
	public:
		GUI(Robot *_robot):robot(_robot){
			logger = Logger::getDefault();
			logger->log("GUI::GUI()", FUNCTION_CALL);
		}
	private:
		void drawSDCardBox(const bool is_logging_available){
			pros::screen::print(TEXT_MEDIUM, 0, "Logging: " + is_logging_available ? "Unavailable": "Available");
		}
		void drawBatteryBox(const double battery_status){
			std::string m = "Battery: " + std::to_string(battery_status);
			pros::screen::print(TEXT_MEDIUM, 1, m.c_str());
		}
		void drawMotors(const HAL *hal){
			auto temps = robot->get_hal()->get_temperatures();
			auto rpms = robot->get_hal()->get_rpms();
			auto torques = robot->get_hal()->get_torques();
			for(int line = 2; line < temps.size() + 2; line++){
				std::string m = "Motor #" + std::to_string(line - 2) + " -> " + std::to_string(rpms[line - 2]).substr(0, 4) + " RPM, " + std::to_string(torques[line - 2]).substr(0,4) + " Nm, " + std::to_string(temps[line - 2]).substr(0,4) + "C";
				pros::screen::print(TEXT_MEDIUM, line, m.c_str());
			}
		}
		Robot *robot;
		Logger *logger;
		int current_line = 0;
};

class RobotController{
	public:
		RobotController(const double _expo, const int _radius, const int _deadzone, Robot *robot): expo(_expo), radius(_radius), deadzone(_deadzone), master(E_CONTROLLER_MASTER), update_task(f){
			Logger *logger = Logger::getDefault();
			logger->log("RobotController::RobotController()", FUNCTION_CALL);
		}
		~RobotController(){
			update_task.remove();
		}
		int get_raw_yaw(){
			return master.get_analog(ANALOG_LEFT_X);
		}
		int get_raw_throttle(){
			return master.get_analog(ANALOG_LEFT_Y);
		}
		int get_scaled_yaw(){
			double input = get_raw_yaw();
			return input * input * expo + input * (1 - expo);
		}
		int get_scaled_throttle(){
			double input = get_raw_throttle();
			return input * input * expo + input * (1 - expo);
		}
		bool get_L1(){
			return master.get_digital(DIGITAL_L1);
		}
		bool get_R1(){
			return master.get_digital(DIGITAL_R1);
		}
		bool get_L2(){
			return master.get_digital(DIGITAL_L2);
		}
		bool get_R2(){
			return master.get_digital(DIGITAL_R2);
		}
		bool get_A(){
			return master.get_digital(DIGITAL_A);
		}
	private:
		double expo;
		int radius;
		int deadzone;
		std::function<void()> f = [](){
			while(true){
				Warning::getDefault()->update();
			}
		};
		Task update_task;
		pros::Controller master;
};

Robot *robot = nullptr;

void initialize() {
	robot = new Robot(0.5 * M_PI, 0.0, 0.0, 50);
	GUI *gui = new GUI(robot);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//Farside red auton (i.e lower right), Basically goes forwards, turns left, and then goes right
	Logger::getDefault()->start();
	robot->get_hal()->set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	//robot->get_hal()->intake_start(true); //start intaking
	Status *status;
	status = robot->goto_pos(1000, 3000, 0, 1200, false);
	while(!status->done){
		delay(20);
		Logger::getDefault()->log("Traveling to 0, 1200", DEBUG_MESSAGE);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);

	status = robot->goto_pos(1000, 3000, -600, 1200, false);
	while(!status->done){
		delay(20);
		Logger::getDefault()->log("Traveling to -600, 1200", DEBUG_MESSAGE);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);	
	status = robot->goto_pos(1000, 3000, 600, 1200, false);
	while(!status->done){
		Logger::getDefault()->log("Traveling to 600, 1200", DEBUG_MESSAGE);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);
	Logger::getDefault()->end();
	//only works blue farside and red farside
	/*
	robot->get_hal()->intake_start(false); //sets the intake to exhaust bc we don't want to intake it (it will be stuck)
	robot->set_throttle(127); //go forwards
	robot->set_yaw(-10); //mild left yaw
	delay(2000); //if it comes in off-axis hopefully it will adjust
	robot->set_yaw(0); //resetting yaw
	robot->set_throttle(-127); //going backwards for 300 ms
	delay(300); 
	robot->set_throttle(127); //go forwards for 400ms
	robot->set_yaw(-5);///if it comes in off-axis hopefully it will adjust
	delay(400);
	robot->set_throttle(0); //reset
	robot->set_yaw(0); //reset
	*/
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	robot->get_hal()->set_brake_mode(E_MOTOR_BRAKE_COAST);
	Logger *logger = Logger::getDefault();
	logger->start();
	logger->log("void opcontrol()", FUNCTION_CALL);
	RobotController controller(0.2, 127, 3, robot);
	while(!controller.get_A()){
		int raw_yaw = controller.get_raw_yaw();
		int raw_throttle = controller.get_raw_throttle();
		robot->set_angle(raw_yaw * (1.0/127.0) * M_PI, false);
		delay(20);
	}
	robot->set_throttle(0);
	robot->set_yaw(0);
	logger->end();
	return;
	/*
	while(true){
		int raw_yaw = controller.get_raw_yaw();
		int raw_throttle = controller.get_raw_throttle();
		robot->set_yaw(raw_yaw);
		robot->set_throttle(raw_throttle);
		if(controller.get_L1()){
			robot->get_hal()->toggle_left_wing(true);
		}
		else{
			robot->get_hal()->toggle_left_wing(false);
		}
		if(controller.get_R1()){
			robot->get_hal()->toggle_right_wing(true);
		}
		else{
			robot->get_hal()->toggle_right_wing(false);
		}
		if(controller.get_A()){
			robot->set_control_point(false);
		}
		else{
			robot->set_control_point(true);
		}
		if(controller.get_L2()){
			robot->get_hal()->intake_start(true);
		}
		else if(controller.get_R2()){
			robot->get_hal()->intake_start(false);
		}
		else{
			robot->get_hal()->intake_stop();
		}
		delay(20);
	}
	*/
	//TBD fix the velocity pid and create a pid test system
}