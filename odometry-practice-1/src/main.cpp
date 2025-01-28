#define _USE_MATH_DEFINES
#include "main.h"
#include "mixer.h"
#include "logger.h"
#include "robot.h"
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
using namespace pros;

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
	robot = new Robot(0.5 * M_PI, 900, 210, 50);
	GUI *gui = new GUI(robot);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	Logger::getDefault()->end();
	Logger::getDefault()->log("logging ended...", DEBUG_MESSAGE);
	Logger::getDefault()->start();
	Logger::getDefault()->log("logging restarted...", DEBUG_MESSAGE);
}

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


void backup(const int del){
	if(del > 0){
		robot->set_throttle(-127);
		robot->set_yaw(0);
		delay(del);
		robot->set_throttle(0);
	}
	if(del < 0){
		robot->set_throttle(127);
		robot->set_yaw(0);
		delay(-del);
		robot->set_throttle(0);
	}

}

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
	//RED FARSIDE
	Status *status;
	int st = 0;
	robot->get_hal()->set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	robot->get_hal()->intake_start(true);
	delay(200);

	status = robot->goto_pos(800, 3000, 900, 410, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);

	status = robot->goto_pos(800, 3000, 1500, 700, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);

	robot->get_hal()->intake_start(false);
	status = robot->goto_pos(1200, 3000, 1500, 950, true);
	st = millis();
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);

	backup(200);

	status = robot->goto_pos(800, 3000, 900, 900, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);

	status = robot->goto_pos(800, 3000, 900, 1200, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);

	//INTAKE
	robot->get_hal()->intake_start(true);
	status = robot->goto_pos(800, 100, 600, 1800, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);
	backup(-200);

	status = robot->goto_pos(800, 3000, 990, 1800, true);
	st = millis();
	while(!status->done){
		delay(20);
		if(millis() - st > 500){
			//EXHAUST
			robot->get_hal()->intake_start(false);
		}
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);
	robot->get_hal()->intake_start(false);
	delay(200);
	backup(200);

	//INTAKE
	robot->get_hal()->intake_start(true);
	status = robot->goto_pos(800, 3000, 210, 1800, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);
	delay(200);

	status = robot->goto_pos(800, 3000, 990, 1800, true);
	st = millis();
	while(!status->done){
		delay(20);
		if(millis() - st > 500){
			//EXHAUST
			robot->get_hal()->intake_start(false);
		}
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);
	//EXHAUST
	robot->get_hal()->intake_start(false);
	delay(200);

	//INTAKE
	robot->get_hal()->intake_start(true);
	status = robot->goto_pos(800, 3000, 210, 1200, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);
	delay(200);

	st = millis();
	status = robot->goto_pos(800, 3000, 990, 1800, true);
	while(!status->done){
		delay(20);
		if(millis() - st > 500){
			//EXHAUST
			robot->get_hal()->intake_start(false);
		}
	}
	robot->acknowledge();
	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);
	//EXHAUST
	robot->get_hal()->intake_start(false);
	delay(200);

	robot->set_throttle(127);
	robot->set_yaw(0);
	delay(200);
	robot->set_throttle(-127);
	robot->set_yaw(0);
	delay(200);
	robot->set_throttle(0);
	robot->set_yaw(0);


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

void writeGigabyte(){
	std::string s(1000000, 'l');
	for(int i = 0; i < 1000; i++){
		Logger::getDefault()->log(s, DEBUG_MESSAGE);
		//delay(100);
	}
}

void opcontrol() {
	robot->get_hal()->set_brake_mode(E_MOTOR_BRAKE_COAST);
	Logger *logger = Logger::getDefault();
	logger->log("void opcontrol()", FUNCTION_CALL);
	RobotController controller(0.2, 127, 3, robot);
	while(true){
		int raw_yaw = controller.get_raw_yaw();
		int raw_throttle = controller.get_raw_throttle();
		robot->set_yaw(raw_yaw);
		robot->set_throttle(raw_throttle);
		if(controller.get_L1()){
			robot->get_hal()->elevator_start(true);
		}
		else{
			robot->get_hal()->elevator_stop();
		}
		if(controller.get_R1()){
			robot->get_hal()->intake_start(true);
		}
		else{
			robot->get_hal()->intake_stop();
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
	//TBD fix the velocity pid and create a pid test system
}