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

void odometry(void *state){
	uint32_t update_delay = 20; //in ms
	int wheel_base_width = 400; //in mm
	int wheel_radius = 100; //in mm
	while(true){
		Motor left(1);
		Motor right(7);
		double left_rpm = left.get_actual_velocity();
		double right_rpm = right.get_actual_velocity();
		if(left_rpm == PROS_ERR_F || right_rpm == PROS_ERR_F){
			cout << "MOTOR FAULT\n";
			return;
		}
		cout << "MOTORS ARE FINE\n";
		double left_spd = left_rpm * (2 * M_PI * wheel_radius) / 60; //in mm/s
		double right_spd = right_rpm * (2 * M_PI * wheel_radius) / 60; //in mm/s
		double dif_spd = left_spd - right_spd;
		double angle_rate = dif_spd / (wheel_base_width / 2); //angle rate in millirad/s
		double forward_spd = (left_spd + right_spd) / 2;
		state_t &state = (state_t &)state;
		state.angle += angle_rate * (update_delay / 1000);
		state.x += forward_spd * cos(state.angle);
		state.y += forward_spd * sin(state.angle);
		cout << state.angle << "\n";
		pros::c::screen_print(TEXT_MEDIUM, 3, "%lf", state.angle);
		delay(update_delay);
	}
}

void initialize() {
	state_t state;
	void *params = &state;
	Task odometry_task(odometry, params);
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
	//pros::Motor left_mtr(1);
	//pros::Motor right_mtr(2);
}
