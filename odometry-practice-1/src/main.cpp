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
		GUI() { //1/30/2025, huge priority
			logger = Logger::getDefault();
			logger->log("GUI::GUI()", FUNCTION_CALL);
		}
		void update(const std::array<double,8> temps, const std::array<double,8> rpms, const std::array<double,8> torques, const std::array<uint32_t,8> faults, const double x, const double y, const double angle) {
			drawBatteryBox(pros::battery::get_capacity());
			drawSDCardBox(Logger::getDefault()->isFileAvailable());
			drawMotors(temps, rpms, torques, faults, x, y, angle);
		};
	private:
		void drawSDCardBox(const bool is_logging_available){
			pros::screen::set_pen(pros::c::COLOR_YELLOW);
			pros::screen::print(TEXT_MEDIUM, 0, (std::string("Logging: ") + std::string(is_logging_available ? "Unavailable": "Available")).c_str());
		}
		void drawBatteryBox(const double battery_status){
			pros::screen::set_pen(pros::c::COLOR_GREEN);
			std::string m = "Battery: " + std::to_string(battery_status);
			pros::screen::print(TEXT_MEDIUM, 1, m.c_str());
		}
		void drawMotors(const std::array<double,8> temps, const std::array<double,8> rpms, const std::array<double,8> torques, std::array<uint32_t,8> faults, const double x, const double y, const double angle){
			for(int line = 2; line < temps.size() + 2; line++){
				if(faults[line - 2] == E_MOTOR_FAULT_NO_FAULTS){
					pros::screen::set_pen(pros::c::COLOR_GREEN);
				}
				else{
					pros::screen::set_pen(pros::c::COLOR_RED);
				}
				
				std::string m = "Motor #" + std::to_string(line - 2) + " -> " + std::to_string(rpms[line - 2]).substr(0, 4) + " RPM, " + std::to_string(torques[line - 2]).substr(0,4) + " Nm, " + std::to_string(temps[line - 2]).substr(0,4) + "C";
				pros::screen::print(TEXT_MEDIUM, line, m.c_str());
			}
			pros::screen::print(TEXT_MEDIUM, temps.size()+2, (std::string("Position -> ") + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(angle)).c_str());
		}
		
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
			return master.get_analog(ANALOG_RIGHT_X);
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
		bool get_L1_rising() {
			return master.get_digital_new_press(DIGITAL_L1);
		}

		bool get_R1(){
			return master.get_digital(DIGITAL_R1);
		}
		bool get_R1_rising() {
			return master.get_digital_new_press(DIGITAL_R1);
		}

		bool get_L2(){
			return master.get_digital(DIGITAL_L2);
		}
		bool get_L2_rising() {
			return master.get_digital_new_press(DIGITAL_L2);
		}

		bool get_R2(){
			return master.get_digital(DIGITAL_R2);
		}
		bool get_R2_rising() {
			return master.get_digital_new_press(DIGITAL_R2);
		}

		bool get_A(){
			return master.get_digital(DIGITAL_A);
		}
		bool get_A_rising() {
			return master.get_digital_new_press(DIGITAL_A);
		}

		bool get_X(){
			return master.get_digital(DIGITAL_X);
		}
		bool get_X_rising() {
			return master.get_digital_new_press(DIGITAL_X);
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
GUI *gui = nullptr;

void initialize() {
	robot = new Robot(0.5 * M_PI, 900, 210, 50);
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
void competition_initialize() {
	gui = new GUI();
}


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
	//1
	/*
	START_POS 214.55, 2400.00
	REVERSE_CONTROL_POINT
	GOTO 900.00, 2400.00 @ 600mm/s DONT DECELERATE
	GOTO 1200.00, 2400.00 @ 200mm/s DECELERATE
	ACTIVATE_CLAMP
	ACTIVATE_INTAKE
	ACTIVATE_ELEVATOR
	GOTO 1200.00, 2700.00 @ 600mm/s DONT DECELERATE
	GOTO 1200.00, 3000.00 @ 100mm/s DECELERATE
	

	Optional subroutine - touch ladder
	GO 1200.00, 2400.00 @ 600mm/s DONT DECELERATE
	GO 1800.00 - (half of robot length), 2400.00 @ 600mm/s DECELERATE

	
	*/
	Status *status;
	robot->get_hal()->set_brake_mode(E_MOTOR_BRAKE_BRAKE); //ENABLE BRAKE MODE
	robot->get_hal()->intake_start(false); //START INTAKE
	robot->get_hal()->elevator_start(false); //START ELEVATOR
	robot->setPos(214.55, 2400.00); //SET START_POS
	robot->set_angle(M_PI);
	robot->set_control_point(false); //REVERSE CONTROL POINT

	status = robot->goto_pos(600, 1, 900, 2400, false);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();

	status = robot->goto_pos(600, 1, 1500, 2400, true);
	while(!status->done){
		delay(20);
	}
	robot->acknowledge();

	robot->get_hal()->toggle_clamp(true);


	Logger::getDefault()->log("DONE", DEBUG_MESSAGE);


	//2024 Over Under Game Auton - Capable of scoring 3 triballs in one fell go - saved here for posterity
	/*
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

void writeGigabyte(){
	std::string s(1000000, 'l');
	for(int i = 0; i < 1000; i++){
		Logger::getDefault()->log(s, DEBUG_MESSAGE);
		//delay(100);
	}
}

void opcontrol() {
	robot->get_hal()->set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	//robot->setPos(214.55, 2400.00); //SET START_POS
	//robot->set_angle(M_PI);
	Logger *logger = Logger::getDefault();
	logger->log("void opcontrol()", FUNCTION_CALL);
	RobotController controller(0.2, 127, 3, robot);
	while(true){
		int raw_yaw = controller.get_raw_yaw();
		int raw_throttle = controller.get_raw_throttle();
		robot->set_yaw(raw_yaw);
		robot->set_throttle(raw_throttle);
		if(controller.get_A_rising()){ //remember default control point is "true" -> TOGGLES
			robot->set_control_point(!robot->get_control_point());
		}

		if(controller.get_L1_rising()){//intake in -> "false" -> TOGGLES
			if(robot->get_hal()->isIntakingIntaking){
				robot->get_hal()->intake_stop();
			}
			else{
				robot->get_hal()->intake_start(false);
			}
		}

		if(controller.get_L2_rising()){ //intake out -> "true" -> TOGGLES
			if(robot->get_hal()->isIntakingIntaking){
				robot->get_hal()->intake_stop();
			}
			else{
				robot->get_hal()->intake_start(true);
			}
		}

		if(controller.get_X_rising()){ //pnuematic toggle, "true" is clamped -> TOGGLES
			robot->get_hal()->toggle_clamp(!robot->get_hal()->clamp_status);
		}

		if(controller.get_R1()){ //lift reverse -> TRIGGER
			robot->get_hal()->elevator_start(true);
		}
		else if(controller.get_R2()){ //lift -> TRIGGER
			robot->get_hal()->elevator_start(false);
		}
		else{
			robot->get_hal()->elevator_stop();
		}
		gui->update(robot->get_hal()->get_temperatures(), robot->get_hal()->get_rpms(), robot->get_hal()->get_torques(), robot->get_hal()->get_motor_faults(), robot->getX(), robot->getY(), robot->get_angle());
		delay(20);
	}
}