#define _USE_MATH_DEFINES
#include "main.h"
#include <vector>
#include <iostream>
#include <cmath>
using namespace pros;
using namespace std;

class State{
	public:
		State(const int left_port, const int right_port, const int _update_freq): left(left_port), right(right_port), update_freq(_update_freq){}
		void update(){
			double update_delay = 1000.0 / update_freq;
			int wheel_base_width = 160; //in mm
			int wheel_radius = 30; //in mm
			uint32_t *tstamp = new uint32_t(millis());
			double left_rev = -1;
			if(!left.is_reversed()){
				left_rev = left.get_raw_position(tstamp) / 1800.0;
			} else{
				left_rev = -left.get_raw_position(tstamp) / 1800.0;
			}
			double right_rev = -1;
			if(right.is_reversed()){
				right_rev = right.get_raw_position(tstamp) / 1800.0;
			} else{
				right_rev = -right.get_raw_position(tstamp) / 1800.0;
			}
			double delta_rev = left_rev - right_rev;
			double dist_bias = delta_rev * (2 * M_PI * wheel_radius);
			double arc_circ = wheel_base_width * 2 * M_PI;
			angle = (dist_bias / arc_circ) * 2 * M_PI;
			double left_rpm = left.get_actual_velocity();
			if(left.is_reversed()){
				left_rpm = -left_rpm;
			}
			double right_rpm = right.get_actual_velocity();
			if(!right.is_reversed()){
				right_rpm = -right_rpm;
			}
			double average_rpm = (left_rpm + right_rpm) / 2;
			velocity = average_rpm * 2 * M_PI * wheel_radius / 60;

			x += cos(angle) * velocity * (update_delay / 1000.0);
			y += sin(angle) * velocity * (update_delay / 1000.0);
			pros::c::screen_print(TEXT_MEDIUM, 6, "X, Y -> %f, %f", x, y);

			cout << *tstamp << " -> " << left_rev << ", " << right_rev << " -> " << 360 * (angle / (2.0 * M_PI)) << ", " << x << ", " << y << "\n";
			delete tstamp;
		}
		double x = 0;
		double y = 0;
		double velocity = 0;
		double angle = 0;
		Motor left;
		Motor right;
	private:
		int update_freq;
};

class Mixer{
	public:
		int yaw = 0; //-127 -> 127
		int throttle = 0; //-127 -> 127
		Mixer(const int left_port, const int right_port): left(left_port), right(right_port, true){}
		void update() {
			double max_rpm = 200;
			double left_target_rpm = (throttle / 127.0) * max_rpm + yaw;
			double right_target_rpm = (throttle / 127.0) * max_rpm - yaw;
			if(left_target_rpm >= 0){
				left.set_reversed(false);
				left.move_velocity(left_target_rpm);
			}
			else{
				left.set_reversed(true);
				left.move_velocity(abs(left_target_rpm));
			}
			if(right_target_rpm >= 0){
				right.set_reversed(true);
				right.move_velocity(right_target_rpm);
			}
			else{
				right.set_reversed(false);
				right.move_velocity(abs(right_target_rpm));
			}
		}
	private:
		Motor left;
		Motor right;
};

class PID{
	public:
		PID(double _p_g, double _i_g, double _d_g, int _update_freq, Mixer *_mixer, State *_state): p_g(_p_g), i_g(_i_g), d_g(_d_g), update_freq(_update_freq), mixer(_mixer), state(_state){}
		void update(){
			double old_angle_error = target_angle - state->angle;
			double old_velocity_error = target_velocity - state->velocity;
			double new_angle_error = target_angle - state->angle;
			double new_velocity_error = target_velocity - state->velocity;
			double yaw_output = 0, throttle_output = 0;
			double p_a, i_a, d_a = 0;
			double p_v, i_v, d_v;
			while(true){
				pros::c::screen_print(TEXT_MEDIUM, 7, "p_g, i_g, d_g -> %f, %f, %f", p_g, i_g, d_g);
				state->update();
				new_angle_error = target_angle - state->angle;

				new_velocity_error = target_velocity - state -> velocity;

				p_a = new_angle_error * p_g;
				i_a += (new_angle_error * (1.0 / update_freq)) * i_g;
				d_a = ((new_angle_error - old_angle_error) / (1.0 / update_freq)) * d_g;
				pros::c::screen_print(TEXT_MEDIUM, 2, "p_a, i_a, d_a -> %f, %f, %f", p_a, i_a, d_a);

				p_v = new_velocity_error * p_g;
				i_v += (new_velocity_error * (1.0 / update_freq)) * i_g;
				d_v = ((new_velocity_error - old_velocity_error) / (1.0 / update_freq)) * d_g;
				pros::c::screen_print(TEXT_MEDIUM, 3, "p_v, i_v, d_v -> %f, %f, %f", p_v, i_v, d_v);

				yaw_output = p_a + i_a + d_a;
				pros::c::screen_print(TEXT_MEDIUM, 4, "PID yaw_output -> %f", yaw_output);

				throttle_output = p_v + i_v + d_v;
				pros::c::screen_print(TEXT_MEDIUM, 5, "PID throttle_output -> %f", throttle_output);

				cout << "PID Yaw -> " << yaw_output << "\n";
				cout << "PID Throttle ->" << throttle_output << "\n";

				mixer->yaw = yaw_output;
				//mixer->throttle = throttle_output;
				mixer->update();

				state->update();
				old_angle_error = target_angle - state->angle;
				old_velocity_error = target_velocity - state->velocity;
				delay(1000.0 / update_freq);
			}
		}
		double p_g, i_g, d_g;
		double target_angle = 0;
		double target_velocity = 0;
		int update_freq; //hz
		Mixer *mixer;
		State *state;
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
/*
void odometry(void *arg){
	state_t *state = (state_t*)arg;
	uint32_t update_delay = 20; //in ms
	int wheel_base_width = 160; //in mm
	int wheel_radius = 30; //in mm
	Motor left(7);
	Motor right(1, true);
	while(true) {
		uint32_t *tstamp = new uint32_t(millis());
		double left_rev = -1;
		if(!left.is_reversed()){
			left_rev = left.get_raw_position(tstamp) / 1800.0;
		} else{
			left_rev = -left.get_raw_position(tstamp) / 1800.0;
		}
		double right_rev = -1;
		if(right.is_reversed()){
			right_rev = right.get_raw_position(tstamp) / 1800.0;
		} else{
			right_rev = -right.get_raw_position(tstamp) / 1800.0;
		}
		double delta_rev = left_rev - right_rev;
		double dist_bias = delta_rev * (2 * M_PI * wheel_radius);
		double arc_circ = wheel_base_width * 2 * M_PI;
		state->angle = (dist_bias / arc_circ) * 2 * M_PI;
		double left_rpm = left.get_actual_velocity();
		if(left.is_reversed()){
			left_rpm = -left_rpm;
		}
		double right_rpm = right.get_actual_velocity();
		if(!right.is_reversed()){
			right_rpm = -right_rpm;
		}
		double average_rpm = (left_rpm + right_rpm) / 2;
		double forward_spd = average_rpm * 2 * M_PI * wheel_radius / 60;
		state->x += cos(state->angle) * forward_spd * (update_delay / 1000.0);
		state->y += sin(state->angle) * forward_spd * (update_delay / 1000.0);
		cout << *tstamp << " -> " << left_rev << ", " << right_rev << " -> " << 360 * (state->angle / (2.0 * M_PI)) << ", " << state->x << ", " << state->y << "\n";
		delete tstamp;
		delay(update_delay);
	}
}
*/
void initialize() {
	//state_t *state = new state_t;
	//Task odometry_task(odometry, state);
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
	int update_freq = 60;
	State *state = new State(7, 1, update_freq * 2);
	Mixer *mixer = new Mixer(7, 1);
	PID *pid = new PID(140, 7, 0, update_freq, mixer, state);
	double angle_rate = 0.5 * M_PI;//rad/s
	double velocity_rate = 2; //mm/s
	double pid_rate = 4; //per second
	auto joystick_update_func = [&angle_rate, &master, pid, mixer, state, update_freq, pid_rate, velocity_rate](){
		while(true){
			pid->target_angle += angle_rate * (1.0 / (update_freq * 4)) * (master.get_analog(ANALOG_LEFT_X) / 127.0);
			//pid->target_velocity += velocity_rate * (1.0 / (update_freq * 4)) * (master.get_analog(ANALOG_LEFT_Y) / 127.0);
			mixer->throttle = (master.get_analog(ANALOG_LEFT_Y));
			pid->i_g += pid_rate * (1.0 / (update_freq * 4)) * (master.get_analog(ANALOG_RIGHT_X) / 127.0);
			pid->d_g += pid_rate * (1.0 / (update_freq * 4)) * (master.get_analog(ANALOG_RIGHT_Y) / 127.0);
			pros::c::screen_print(TEXT_MEDIUM, 0, "PID target_angle -> %f", pid->target_angle);
			cout << "PID target_angle -> " << pid->target_angle << "\n";
			pros::c::screen_print(TEXT_MEDIUM, 1, "Actual angle -> %f", state->angle);
			cout << "Actual angle -> " << state->angle << "\n";
			delay(1000.0 / (update_freq * 4));
		}
	};
	Task joystick_update_task(joystick_update_func);
	pid->update();
}
