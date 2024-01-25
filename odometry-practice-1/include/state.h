#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#define _USE_MATH_DEFINES
#undef __ARM_NEON__
#undef __ARM_NEON
#include "api.h"
#include "hal.h"
#include "mixer.h"
#include <mutex>
#include <utility>
#include <cmath>
#include "Eigen/Dense"
using namespace pros;
using namespace pros::literals;


static std::string toString(const Eigen::MatrixXd& mat){
    std::stringstream ss;
    ss << mat;
    std::string s = ss.str();
    std::replace(s.begin(), s.end(), '\n', ' ');
    return s;
}

class State{
	public:
		State(HAL *hal, Mixer *mixer, const double start_x, const double start_y, const double start_angle, const int _update_freq);
		void update();
		double getX(){
			std::lock_guard lock(mutex);
			return new_state.first(0);
		}
		void setX(const double new_x){
			std::lock_guard lock(mutex);
			new_state.first(0) = new_x;
		}
		double getY(){
			std::lock_guard lock(mutex);
			return new_state.first(1);
		}
		void setY(const double new_y){
			std::lock_guard lock(mutex);
			new_state.first(1) = new_y;
		}
		double getVelocity(){
			std::lock_guard lock(mutex);
			return new_state.first(2);
		}
		double getLeftRPM(){
			std::lock_guard lock(mutex);
			return hal->get_left_velocity();
		}
		double getRightRPM(){
			std::lock_guard lock(mutex);
			return hal->get_right_velocity();
		}
		double getAngle(){
			std::lock_guard lock(mutex);
			return new_state.first(3);
		}
		void setAngle(const double new_angle){
			std::lock_guard lock(mutex);
			new_state.first(3) = new_angle;
		}
		void set_control_point(const bool b){
			std::lock_guard lock(mutex);
			isForward = b;
		}
		const double wheel_radius = 41.275;
		const double wheelbase_diameter = 300.0;
	private:
			/*
			state is often written as x
			variance is often written as P
			F is the state transition function. It computes the next state from the previous state. 
			Q is the process covariance matrix.
			B and u are new, they are used to model control inputs (motor movements) to the system.
			u represents the raw control input, such as motor voltage.
			B tranlates that into a state variable
			*/
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> predict(const std::pair<Eigen::Vector4d, Eigen::Matrix4d> &in){
			//first is prev_state, second is variance
			std::pair<Eigen::Vector4d, Eigen::Matrix4d> res;
			Eigen::Vector4d u{mixer->getThrottle(), mixer->getYaw(), 0, 0};
			B(2, 0) = 3000.0 / (127.0 * update_freq);
			B(3, 1) = (2.0 * M_PI) / (127.0 * update_freq);
			F(0, 2) = cos(in.first(3)) / update_freq;
			F(1, 2) = sin(in.first(3)) / update_freq;
			res.first = F * in.first + B * u;
			res.second = F * in.second * F.transpose() + Q;
			return res;
		}
		/*
		H is the measurement function. It is the measurement analog to B, 
		it maps state variables, like position or velocity, to raw sensor values, such as encoder ticks. 
		It seems counterintuitive at first, but we are not converting the measurement to our state variables at this point. 
		z and R are the measurement mean and covariance, respectively. y is the residual, or the difference between the measurement and prediction. K is the Kalman Gain. I is the identity matrix, which consists of 1s in the diagonal of the matrix.
		*/
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> update(const std::pair<Eigen::Vector4d, Eigen::Matrix4d> &in){
  			//first is state, second is variance
			double v = ((getLeftRPM() + getRightRPM()) / 2.0) * 2.0 * M_PI * wheel_radius / (60.0 * static_cast<double>(update_freq));
			double theta_imu = start_angle - (hal->get_imu().get_rotation() / 180.0) * M_PI;
			double l_ct = hal->get_left_rotations();
			double r_ct = hal->get_right_rotations();
			//double theta_odom = start_angle + 2 * M_PI * ((r_ct - l_ct) * wheel_radius) / (wheelbase_diameter);
			double theta_odom = theta_imu;
			double dx = v * cos(theta_imu), dy = v * sin(theta_imu);
			//z << z(0) + dx, z(1) + dy, v, theta_imu, theta_odom;
			z(0) += dx;
			z(1) += dy;
			z(2) = v;
			z(3) = theta_imu;
			z(4) = theta_odom;
			Logger::getDefault()->log("Z -> " + toString(z), DEBUG_MESSAGE);
			std::pair<Eigen::Vector4d, Eigen::Matrix4d> res;
			Eigen::Vector<double, 5> y;
			y = z - H * in.first;
  			Eigen::Matrix<double, 4,5> K;//should be 4x5
			K = in.second * H.transpose() * (H * in.second * H.transpose() + R).inverse();
  			res.first = in.first + K * y;
  			res.second = (I - K * H) * in.second;
			return res;
		}
		bool isForward = true;
		const double start_angle = 0;
		const int update_freq;
		const double update_delay = 1000.0 / update_freq;
		//F * prev_state = new_state
		//Identity Matrix
		//H * state = measurement
		//B * mixer_inputs is used by F * prev_state to factor for acceleration and angular rate caused by control inputs. Eg. Holding full right stick
		//R -> process noise covariance matrix
		//Q -> matrix that represents the variance between step to step
		//P -> The initial variance, represents an x and y uncertainty of 10mm and 5 deg
		Eigen::Matrix4d F = Eigen::Matrix4d::Identity(), B = Eigen::Matrix4d::Zero(), I = Eigen::Matrix4d::Identity();
		const Eigen::Matrix4d Q{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 20.0, 0}, {0, 0, 0, 0.02}};
		Eigen::Matrix4d P{{10.0, 0, 0, 0}, {0, 10.0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; //2.5 deg uncertainty and 10mm x and y uncertainty
		Eigen::Matrix<double, 5, 4> H; //5 rows, 4 cols
		Eigen::Matrix<double, 5, 5> R; //5 rows, 5 cols
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> prev_state;
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> new_state;
		Eigen::Vector<double, 5> z;
		HAL *hal;
		Mixer *mixer;
		Mutex mutex;
};