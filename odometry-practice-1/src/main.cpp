#define _USE_MATH_DEFINES
#include "main.h"
#include "mixer.h"
#include "state.h"
#include "pid.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <queue>
using namespace pros;
using namespace std;

struct MotionCmd{
	MotionCmd(const double _max_velocity, const double _a_time, const double _end_x, const double _end_y): max_velocity(_max_velocity), a_time(_a_time), end_x(_end_x), end_y(_end_y){}
	double max_velocity;
	double a_time;
	double end_x;
	double end_y;
};

class LogItem{
	public:
		LogItem(const std::string _message, const int _timestamp): message(_message), timestamp(_timestamp){}
		std::string message;
		int timestamp;
};

class Robot {
	public:
		Robot(const double start_angle, const double start_x, const double start_y, const int _update_freq) : update_freq(_update_freq){
			state = new State(1, 2, 3, 8, 9, 10, 7, start_x, start_y, start_angle, update_freq);
			state->initializeImu();
			mixer = new Mixer(1, 2, 3, 8, 9, 10);
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
			//delete state;
			delete mixer;
			delete pid;
			delete goto_task;
			delete pid_task;
		}
		void set_angle(const double new_angle, const bool blocking){
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
			//enable pid
			pid->changeRunningState(true);
			pid->setTargetVelocity(new_velocity);
			if(blocking){
				//wait until the robot reaches the target angle, check every 5ms
			}
		}
		void set_yaw(const int new_yaw){
			//disable pid
			pid->changeRunningState(false);
			mixer->setYaw(new_yaw);
			mixer->update();
		}
		void set_throttle(const int new_throttle){
			//disable pid
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
		double update_freq = -1;
		State *state; // = new State(7, 1, 2, update_freq * 2);
		Mixer *mixer; // = new Mixer(7, 1);
		PID *pid; // = new PID(140, 0, 1, update_freq, mixer, state)
		std::queue<MotionCmd> queue;
		Task *goto_task;
		Task *pid_task;
		Task *state_task;
		Mutex mutex;
		double acc_dist = 10;
		std::function<void()> goto_func = [this](){
			while(pros::Task::notify_take(true, TIMEOUT_MAX)){ //timeout
					//execute all commands in queue
					mutex.take(TIMEOUT_MAX);
					while(!queue.empty()){
						MotionCmd cmd = queue.front();
						queue.pop();
						mutex.give();
						//execute motioncmd somehow
						double cur_x = 0;//state->getX();
						double cur_y = 0; //state->getY();
						double cur_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						double old_dist = get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y);
						while(get_distance(cur_x, cur_y, cmd.end_x, cmd.end_y) > acc_dist){
							cur_x = 0;//state->getX();
							cur_y = 0;//state->getY();
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

class GUI{
	public:
		GUI(Robot *_robot):robot(_robot){
			auto disp_func = [this](){
				while(true){
					pros::c::screen_set_eraser(COLOR_BLACK);
					pros::c::screen_erase();
					pros::c::screen_print(TEXT_MEDIUM, 0, "Actual Yaw - > %f\n", robot->state->getAngle());
					pros::c::screen_print(TEXT_MEDIUM, 0, "Target Yaw - > %f\n", robot->pid->getTargetAngle());
					pros::c::screen_set_pen(COLOR_GREEN);
					pros::c::screen_draw_rect(0, 25, 254, 80);
					pros::c::screen_set_pen(COLOR_RED);
					pros::c::screen_draw_rect(127, 25, robot->mixer->getYaw() + 127, 80);
					delay(20);
				}
			};
			disp_task = new Task(disp_func);
		}
		~GUI(){
			disp_task->remove();
		}
	private:
		Robot *robot;
		Task *disp_task;
};

class RobotController{
	public:
		RobotController(const double _expo, const int _radius, const int _deadzone): expo(_expo), radius(_radius), deadzone(_deadzone), master(E_CONTROLLER_MASTER){}
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
			double raw_yaw = master.get_analog(ANALOG_LEFT_Y);
			double raw_throttle = master.get_analog(ANALOG_LEFT_Y);
			mutex.give();
			double max_yaw = sqrt(pow(radius, 2) - pow(raw_throttle, 2));
			//double max_throttle = sqrt(pow(radius, 2) - pow(raw_yaw, 2));
			return raw_yaw;
		}
		int get_scaled_throttle(){
			mutex.take(TIMEOUT_MAX);
			double raw_yaw = master.get_analog(ANALOG_LEFT_Y);
			double raw_throttle = master.get_analog(ANALOG_LEFT_Y);
			mutex.give();
			//double max_yaw = sqrt(pow(radius, 2) - pow(raw_throttle, 2));
			double max_throttle = sqrt(pow(radius, 2) - pow(raw_yaw, 2));

			return raw_throttle;
		}
	private:
		double expo;
		int radius;
		int deadzone;
		Mutex mutex;
		pros::Controller master;
};

class Logger{
	public:
		Logger(){
			//TBD: Determine log system, simply log to cout? Or log to a file?
			//How would I make the log truly global? Possibly make it a global variable, but then it would be stuck in main? Pass pointers to log using functions?
			
			/*
			00:00:00:001 -> Robot initialized...
			00:00:00:002 -> Mixer intialized...
			00:00:00:003 -> State initialized...
			00:00:00:004 -> PID initialized...
	    	00:00:00:009 -> IMU Calibration started....
			00:00:02:300 -> IMU Calibration finished....
			*/
			log("Logger initialized...");
		}
		void log(const std::string message){
			mutex.take(TIMEOUT_MAX);
			items.push(LogItem(message, millis()));
			while(!items.empty()){
				cout << to_timestamp(items.front().timestamp) << " -> " << items.front().message << "\n";
				items.pop();
			}
			mutex.give();
		}
	private:
		string to_timestamp(const int timestamp){
			int hh = (timestamp / 3600000) % 24; //hour
			int mm = (timestamp / 60000) % 60;
			int ss = (timestamp / 1000) % 60;
			int ms = timestamp % 1000;
			return std::to_string(hh) + ":" + std::to_string(mm) + ":" + std::to_string(ss) + ":" + std::to_string(ms);
		}
		Mutex mutex;
		std::queue<LogItem> items;
};

Logger logger;
Robot robot(0.5 * M_PI, 0.0, 0.0, 50);

void initialize() {}

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
	//TODO: Put autonomous code
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
	cout << "Beginning OpControl\n";
	logger.log("Beginning Opcontrol");
	//TODO: ADD OPCONTROL CODE
	//robot.update();
	logger.log("Ran robot update");
	RobotController controller(0.2, 127, 3);
	logger.log("Created RobotController");
	while(true){
		int raw_yaw = controller.get_raw_yaw();
		int raw_throttle = controller.get_raw_throttle();
		robot.set_yaw(raw_yaw);
		robot.set_throttle(raw_throttle);
		logger.log(to_string(robot.get_angle()) + ", " + to_string(robot.get_velocity()));
		//logger.log("hello");
		delay(20);
	}
}