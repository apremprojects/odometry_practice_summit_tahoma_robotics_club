#define _USE_MATH_DEFINES
#include "main.h"
#include "mixer.h"
#include "state.h"
#include "pid.h"
#include <vector>
#include <iostream>
#include <cmath>
using namespace pros;
using namespace std;

class Robot{
	public:
		Robot(const double start_angle, const double start_x, const double start_y, const double _update_freq) : update_freq(_update_freq){
			state = new State(7, 1, 2, update_freq);
			mixer = new Mixer(7, 1);
			pid = new PID(140, 0, 3, update_freq, mixer, state);
		}
		~Robot(){
			delete state;
			delete mixer;
			delete pid;
		}
		void set_angle(const double new_angle, const bool blocking){
			pid->target_angle = new_angle;
			if(blocking){
				//wait until the robot reaches the target angle, check every 20ms
				while(abs(get_angle() - new_angle) > 0.1){
					delay(20);
				}
			}
			return;
		}
		void set_velocity(const double new_velocity, const bool blocking){
			pid->target_velocity = new_velocity;
			if(blocking){
				//wait until the robot reaches the target angle, check every 5ms
			}
		}
		double get_angle(){
			return state->angle;
		}
		double get_velocity(){
			return state->velocity;
		}
		void goto_pos(const double max_velocity, const double a_time, const double end_x, const double end_y, const bool blocking){
			auto goto_func = [max_velocity, a_time, end_x, end_y, this](){
				this->set_velocity(max_velocity, false);
				this->set_angle(get_angle(this->state->x, this->state->y, end_x, end_y), true);	
				while(get_distance(this->state->x, this->state->y, end_x, end_y) > 10){
					this->set_angle(get_angle(this->state->x, this->state->y, end_x, end_y), true);
					delay(20);
				}
				this->set_velocity(0, false);
				return;
			};
			if(!blocking){
				Task goto_task(goto_func);
			}
			else{
				goto_func();
			}
		}
		void update(){
			pid->update();
		}
		double get_angle(const double cur_x, const double cur_y, const double end_x, const double end_y){
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

};

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

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
void autonomous() {}

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	int update_freq = 50;
	/*PID Tuning guide
	-Set all gains to 0
	-Increase P till the robot oscillates
	-Increase D to damp the oscillation out
	-Increase I to reduce overshoot
	*/
	double angle_rate = 2 * M_PI;//rad/s
	double max_acceleration = 1400; //mm/s
	double max_velocity = 700;
	double pid_rate = 4; //per second
	Robot *robot = new Robot(0, 0, 0, update_freq);
	auto update_func = [&master, max_acceleration, angle_rate, robot](){
		delay(100);
		robot->goto_pos(600, 1000, 1000, 0, true);
		robot->goto_pos(600, 1000, 1000, 1000, true);
		robot->goto_pos(600, 1000, 0, 1000, true);
		robot->goto_pos(600, 1000, 0, 0, true);
		//robot->set_velocity(2000, false);
		//delay(2000);
		//robot->set_angle(M_PI / 2.0, false);
		//delay(2000);
		//robot->set_velocity(0, false);
		return;
	};

	Task update_task(update_func);
	robot->update();
	//delete robot;
}