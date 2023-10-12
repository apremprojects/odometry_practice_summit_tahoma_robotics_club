#define _USE_MATH_DEFINES
#include "main.h"
#include <vector>
#include <iostream>
#include <cmath>
using namespace pros;
using namespace std;

class State{
	public:
		State(const int left_port, const int right_port, const int imu_port, const int _update_freq): left(left_port), right(right_port), imu(imu_port), update_freq(_update_freq){
			imu.reset(true);
			imu.set_rotation(0);
		}
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
			angle = 2 * M_PI * (imu.get_rotation() / 360.0);
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

			//cout << *tstamp << " -> " << left_rev << ", " << right_rev << " -> " << 360 * (angle / (2.0 * M_PI)) << ", " << x << ", " << y << "\n";
			delete tstamp;
		}
		double x = 0;
		double y = 0;
		double velocity = 0;
		double angle = 0;
		Motor left;
		Motor right;
		pros::Imu imu;
	private:
		int update_freq;
};

class Mixer{
	public:
		int yaw = 0; //-127 -> 127
		int throttle = 0; //-127 -> 127
		Mixer(const int left_port, const int right_port): left(left_port), right(right_port, true){}
		void update() {
			if(yaw > 127){
				yaw = 127;
			}
			else if(yaw < -127){
				yaw = -127;
			}
			if(throttle > 127){
				throttle = 127;
			}
			else if(throttle < -127){
				throttle = -127;
			}
			double max_rpm = 200;
			double left_target_rpm = (throttle / 127.0) * max_rpm + yaw;
			double right_target_rpm = (throttle / 127.0) * max_rpm - yaw;
			if(left_target_rpm > 200){
				left_target_rpm = 200 + yaw;
			}
			else if(left_target_rpm < -200){
				left_target_rpm = -200 + yaw;
			}
			if(right_target_rpm > 200){
				right_target_rpm = 200 - yaw;
			}
			else if(right_target_rpm < -200){
				right_target_rpm - -200 - yaw;
			}
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
			double old_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);
			double new_angle_error = target_angle - state->angle;
			double new_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);
			double yaw_output = 0, throttle_output = 0;
			double p_a, i_a, d_a = 0;
			double p_v, i_v, d_v;
			while(true){
				pros::c::screen_print(TEXT_MEDIUM, 7, "p_g, i_g, d_g -> %f, %f, %f", p_g, i_g, d_g);
				state->update();
				pros::c::screen_print(TEXT_MEDIUM, 8, "Target Velocity -> %f", target_velocity);
				pros::c::screen_print(TEXT_MEDIUM, 9, "Actual Velocity -> %f", state->velocity);
				new_angle_error = target_angle - state->angle;

				new_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);

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

				//cout << "PID Yaw -> " << yaw_output << "\n";
				//cout << "PID Throttle ->" << throttle_output << "\n";

				mixer->yaw = yaw_output;
				mixer->throttle = throttle_output;
				mixer->update();
				state->update();
				old_angle_error = target_angle - state->angle;
				old_velocity_error = (target_velocity / 1000.0) - (state->velocity / 1000.0);
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

class Robot{
	public:
		Robot(const double start_angle, const double start_x, const double start_y, const double _update_freq) : update_freq(_update_freq){
			state = new State(7, 1, 2, update_freq);
			mixer = new Mixer(7, 1);
			pid = new PID(140, 0.4, 3, update_freq, mixer, state);
		}
		~Robot(){
			delete state;
			delete mixer;
			delete pid;
		}
		void set_angle(const double new_angle, const bool blocking){
			pid->target_angle = new_angle;
			if(blocking){
				//wait until the robot reaches the target angle, check every 5ms
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
					//cout << "Cruising\n";
					this->set_angle(get_angle(this->state->x, this->state->y, end_x, end_y), true);
					delay(20);
				}
				this->set_velocity(0, false);
				return;
			};
			Task goto_task(goto_func);
		}
		void update(){
			pid->update();
		}
	private:
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
	auto update_func = [robot, &master](){
		delay(1000);
		while(true){
			robot->set_angle(2 * M_PI * (master.get_analog(ANALOG_LEFT_X) / 127.0), false);
			robot->set_velocity(700 * (master.get_analog(ANALOG_LEFT_Y) / 127.0), false);
		}
		//robot->set_velocity(2000, false);
		//delay(2000);
		//robot->set_angle(M_PI / 2.0, false);
		//delay(2000);
		//robot->set_velocity(0, false);
		return;
	};
	Task update_task(update_func);
	robot->update();
	delete robot;
}