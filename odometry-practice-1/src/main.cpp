#define _USE_MATH_DEFINES
#include "main.h"
#include <vector>
#include <iostream>
#include <cmath>
using namespace pros;
using namespace std;

struct state_t{
	double x = 0;
	double y = 0;
	double angle = 0;
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
		double angle = (dist_bias / arc_circ) * 360.0;
		cout << *tstamp << " -> " << left_rev << ", " << right_rev << " -> " << angle << "\n";
		delete tstamp;
		delay(update_delay);
	}
	/*while(true){
		double left_rpm = left.get_actual_velocity();
		double right_rpm = right.get_actual_velocity();
		if(left_rpm == PROS_ERR_F || right_rpm == PROS_ERR_F){
			cout << "MOTOR FAULT\n";
			return;
		}
		cout << "MOTORS ARE FINE\n";

		if (state) {
		    cout << "State ARE FINE\n";
		}

		double left_spd = left_rpm * (2 * M_PI * wheel_radius) / 60; //in mm/s
		double right_spd = right_rpm * (2 * M_PI * wheel_radius) / 60; //in mm/s
		double dif_spd = left_spd - right_spd;
		double angle_rate = dif_spd / (wheel_base_width / 2); //angle rate in millirad/s
		double forward_spd = (left_spd + right_spd) / 2;

		state->angle += angle_rate * (update_delay / 1000.0);
		state->x += forward_spd * cos(state->angle);
		state->y += forward_spd * sin(state->angle);
		cout << "Angle -> " << (state->angle / (2.0 * M_PI)) * 360.0 << "\n";
		cout << "Left RPM -> " << left_rpm << "\n";
		cout << "Right RPM -> " << right_rpm << "\n";
		pros::c::screen_print(TEXT_MEDIUM, 3, "%lf", (state->angle / (2.0 * M_PI)) * 360.0);
		delay(update_delay);
	}*/
}

void initialize() {
	state_t *state = new state_t;
	Task odometry_task(odometry, state);
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
	Motor left(7);
	Motor right(1, true);
	while(true){
		double yaw_mix = master.get_analog(ANALOG_LEFT_X);
		double throttle = master.get_analog(ANALOG_LEFT_Y); //-127 to 127
		double max_rpm = 200;
		//cout << "Yaw mix -> " << yaw_mix << "\n";
		//cout << "Throttle -> " << throttle << "\n";
		double left_target_rpm = (throttle / 127.0) * max_rpm + yaw_mix;
		double right_target_rpm = (throttle / 127.0) * max_rpm - yaw_mix;
		if(left_target_rpm >= 0){
			left.set_reversed(false);
			left.move_velocity(left_target_rpm);
			//cout << "LEFTRPM >= 0\n";
		}
		else{
			left.set_reversed(true);
			left.move_velocity(abs(left_target_rpm));
			//cout << "LEFTRPM < 0\n";
		}
		if(right_target_rpm >= 0){
			right.set_reversed(true);
			right.move_velocity(right_target_rpm);
			//cout << "RIGHTRPM >= 0\n";
		}
		else{
			right.set_reversed(false);
			right.move_velocity(abs(right_target_rpm));
			//cout << "RIGHTRPM < 0\n";
		}
		delay(20);
	}
}
