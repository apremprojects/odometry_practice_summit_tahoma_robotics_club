#define _USE_MATH_DEFINES
#include "main.h"
#include "mixer.h"
#include "logger.h"
#include "hal.h"
#include "state.h"
#include "pid.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <queue>
using namespace pros;

struct MotionCmd{
	MotionCmd(const double _max_velocity, const double _a_time, const double _end_x, const double _end_y): max_velocity(_max_velocity), a_time(_a_time), end_x(_end_x), end_y(_end_y){}
	double max_velocity;
	double a_time;
	double end_x;
	double end_y;
};

class Robot {
	public:
		Robot(const double start_angle, const double start_x, const double start_y, const int _update_freq) : update_freq(_update_freq){
			logger = Logger::getDefault();
			logger->log("Robot::Robot()", FUNCTION_CALL);
			hal = new HAL(16, 5, 4, 1, 2, 3, 8, 9, 10, 7);
			state = new State(hal, start_x, start_y, start_angle, update_freq);
			mixer = new Mixer(hal);
			pid = new PID(140, 0, 3, update_freq, mixer, state);
			pid_task = new Task([this](){pid->update();});
			state_task = new Task([this](){
				while(true){
					state->update();
					delay(1000/update_freq);
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
			if(blocking){
				//wait until the robot reaches the target angle, check every 20ms
				while(abs(get_angle() - new_angle) > 0.1){
					delay(20);
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
				//wait until the robot reaches the target angle, check every 5ms
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
		void goto_pos(const double max_velocity, const double a_time, const double end_x, const double end_y, const bool blocking){
			MotionCmd cmd(max_velocity, a_time, end_x, end_y);
			mutex.take(TIMEOUT_MAX);
			queue.push(cmd);
			mutex.give();
			goto_task->notify();
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
			mutex.take(TIMEOUT_MAX);
			isForward = b;
			mixer->set_control_point(isForward);
			state->set_control_point(isForward);
			mutex.give();
		}
		HAL* get_hal(){
			return hal;
		}
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
					//execute all commands in queue
					mutex.take(TIMEOUT_MAX);
					while(!queue.empty()){
						MotionCmd cmd = queue.front();
						logger->log(std::to_string(cmd.end_x) + " " + std::to_string(cmd.end_y) + " " + std::to_string(cmd.max_velocity) + " " + std::to_string(cmd.a_time), TARGET_POS_UPDATE);
						queue.pop();
						mutex.give();
						//execute motioncmd somehow
						double cur_x = state->getX();
						double cur_y = state->getY();
						double cur_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						double old_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						while(get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y) > acc_dist){
							cur_x = state->getX();
							cur_y = state->getY();
							cur_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
							double angle = get_angle_between(cur_x, cur_y, cmd.end_x, cmd.end_y);
							set_angle(angle, false);
							if(abs(angle - state->getAngle()) < 0.5 * M_PI) {
								set_velocity(cmd.max_velocity, false);
							}
							else{
								set_velocity(0, false);
							}
						}
						set_velocity(0, false);
						delay(20);
						old_dist = cur_dist;
					}
					mutex.give();
			}
		};
};

/*Max 5 rows

SDCardStatus | Battery
Motor1 | Motor2
Motor3 | Motor4
Motor5 | Motor6
IntakeStatus | Catapult
*/

class GUI{
	public:
		GUI(Robot *_robot):robot(_robot){
			logger = Logger::getDefault();
			logger->log("GUI::GUI()", FUNCTION_CALL);
			auto disp_func = [this](){
				while(true){
					//remember res is 480x240
					drawSDCardBox(logger->isFileAvailable());
					drawBatteryBox(pros::battery::get_capacity());
					drawMotors(robot->get_hal());
					//handle warnings, send stuff to controller
					delay(50);
				}
			};
			disp_task = new Task(disp_func);
		}
		~GUI(){
			disp_task->remove();
		}
	private:
		void drawSDCardBox(const bool is_logging_available){
			pros::screen::set_pen(COLOR_GREEN);
			if(!is_logging_available){
				pros::screen::set_pen(COLOR_RED);
			}
			pros::screen::draw_rect(0, 0, 240, 30);//draw a rect for the SDCard Status Box
			pros::screen::set_pen(COLOR_WHITE);
			pros::screen::print(TEXT_LARGE, 0, 0, "SDCard Status: " + is_logging_available ? "Available": "Unavailable");
		}
		void drawBatteryBox(const double battery_status){
			pros::screen::set_pen(COLOR_GREEN);
			if(battery_status <= 33){
				pros::screen::set_pen(COLOR_RED);
			}
			else if(battery_status <= 66){
				pros::screen::set_pen(COLOR_YELLOW);
			}
			pros::screen::draw_rect(240, 0, 480, 30);//draw a rect for the Battery Status Box
			pros::screen::set_pen(COLOR_WHITE);
			pros::screen::print(TEXT_LARGE, 0, 0, "Battery: " + std::to_string(battery_status));
		}
		void drawMotors(const HAL *hal){
			return;
		}
		Robot *robot;
		Logger *logger;
		int current_line = 0;
		Task *disp_task;
};

class RobotController{
	public:
		RobotController(const double _expo, const int _radius, const int _deadzone): expo(_expo), radius(_radius), deadzone(_deadzone), master(E_CONTROLLER_MASTER){
			Logger *logger = Logger::getDefault();
			logger->log("RobotController::RobotController()", FUNCTION_CALL);
		}
		int get_raw_yaw(){
			mutex.take(TIMEOUT_MAX);
			int v = master.get_analog(ANALOG_LEFT_X);
			mutex.give();
			return v;
		}
		int get_raw_throttle(){
			mutex.take(TIMEOUT_MAX);
			int v = master.get_analog(ANALOG_LEFT_Y);
			mutex.give();
			return v;
		}
		int get_scaled_yaw(){
			mutex.take(TIMEOUT_MAX);
			int raw_yaw = get_raw_yaw();
			mutex.give();
			return raw_yaw;
		}
		int get_scaled_throttle(){
			mutex.take(TIMEOUT_MAX);
			int raw_throttle = get_raw_throttle();
			mutex.give();
			return raw_throttle;
		}
		bool getL1(){
			mutex.take(TIMEOUT_MAX);
			bool L1 = master.get_digital(DIGITAL_L1);
			mutex.give();
			return L1;
		}
		bool getR1(){
			mutex.take(TIMEOUT_MAX);
			bool R1 = master.get_digital(DIGITAL_R1);
			mutex.give();
			return R1;
		}
	private:
		double expo;
		int radius;
		int deadzone;
		Mutex mutex;
		pros::Controller master;
};

Robot *robot = nullptr;
void initialize() {
	Logger *logger = Logger::getDefault();
	robot = new Robot(0.5 * M_PI, 0.0, 0.0, 50);
	logger->log("void initialize()", FUNCTION_CALL);
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
	Logger *logger = Logger::getDefault();
	logger->log("void autonomous()", FUNCTION_CALL);
	robot->set_yaw(0);
	robot->set_throttle(0);
	robot->set_throttle(127);
	delay(1000);
	robot->set_throttle(0);
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
	Logger *logger = Logger::getDefault();
	logger->log("void opcontrol()", FUNCTION_CALL);
	RobotController controller(0.2, 127, 3);
	while(true){
		int raw_yaw = controller.get_raw_yaw();
		int raw_throttle = controller.get_raw_throttle();
		robot->set_yaw(raw_yaw);
		robot->set_throttle(raw_throttle);
		if(controller.getL1()){
			robot->get_hal()->toggle_left_wing(true);
		}else{
			robot->get_hal()->toggle_left_wing(false);
		}
		if(controller.getR1()){
			robot->get_hal()->toggle_right_wing(true);
		}else{
			robot->get_hal()->toggle_right_wing(false);
		}
		delay(20);
	}
}