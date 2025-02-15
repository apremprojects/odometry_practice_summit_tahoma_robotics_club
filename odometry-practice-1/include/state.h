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
		void setPos(const double new_x, const double new_y){
			std::lock_guard lock(mutex);
			z(0) = new_x;
			z(1) = new_y;
			z(2) = 0;
			prev_state.first(0) = new_x;
			prev_state.first(1) = new_y;
			prev_state.first(2) = 0;
			new_state.first(0) = new_x;
			new_state.first(1) = new_y;
			new_state.first(2) = 0;
		}
		void setAngle(const double new_angle){
			std::lock_guard lock(mutex);
			start_angle = new_angle + (hal->get_imu().get_rotation() / 180.0) * M_PI;
		}
		double getX(){
			std::lock_guard lock(mutex);
			return new_state.first(0);
		}
		double getY(){
			std::lock_guard lock(mutex);
			return new_state.first(1);
		}
		double getVelocity(){
			std::lock_guard lock(mutex);
			return new_state.first(2);
		}
		double getLeftRPM(){
			std::lock_guard lock(mutex);
			if(isForward){
				return hal->get_left_velocity();
			}
			return hal->get_right_velocity();
		}
		double getRightRPM(){
			std::lock_guard lock(mutex);
			if(isForward){
				return hal->get_right_velocity();
			}
			return hal->get_left_velocity();
		}
		double getAngle(){
			std::lock_guard lock(mutex);
			return new_state.first(3);
		}
		void set_control_point(const bool b){
			std::lock_guard lock(mutex);
			isForward = b;
		}
		const double wheel_radius = 260 / (2 * M_PI);
		const double wheelbase_diameter = 320.0;
	private:
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> predict(const std::pair<Eigen::Vector4d, Eigen::Matrix4d> in){
			//Logger::getDefault()->log("IN -> " + toString(in.first), DEBUG_MESSAGE);
			std::pair<Eigen::Vector4d, Eigen::Matrix4d> res;
			Eigen::Vector4d u{mixer->getThrottle(), mixer->getYaw(), 0, 0};
			B(2, 0) = 3000.0 / (127.0 * update_freq);
			B(3, 1) = (2.0 * M_PI) / (127.0 * update_freq);
			F(0, 2) = cos(in.first(3)) / update_freq;
			F(1, 2) = sin(in.first(3)) / update_freq;
			res.first = F * in.first + B * u;
			res.second = F * in.second * F.transpose() + Q;
			//Logger::getDefault()->log("predict " + toString(res.first) + ", " + toString(res.second), DEBUG_MESSAGE);
			return res;
		}
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> update(const std::pair<Eigen::Vector4d, Eigen::Matrix4d> in){
			double v = ((getLeftRPM() + getRightRPM()) / 2.0) * 2.0 * M_PI * wheel_radius / (60.0 * static_cast<double>(update_freq));
			double theta_imu = getRawAngle();
			double l_ct = hal->get_left_rotations();
			double r_ct = hal->get_right_rotations();
			//double theta_odom = start_angle + 2 * M_PI * ((r_ct - l_ct) * wheel_radius) / (wheelbase_diameter);
			double theta_odom = theta_imu;
			double dx = v * cos(theta_imu), dy = v * sin(theta_imu);
			if(isForward){
				z(0) += dx;
				z(1) += dy;
			}
			else{
				z(0) -= dx;
				z(1) -= dy;
			}
			z(2) = v;
			z(3) = theta_imu;
			z(4) = theta_odom;
			//Logger::getDefault()->log("raw sensor input -> " + toString(z), DEBUG_MESSAGE);
			std::pair<Eigen::Vector4d, Eigen::Matrix4d> res;
			Eigen::Vector<double, 5> y;
			y = z - H * in.first;
  			Eigen::Matrix<double, 4,5> K;//should be 4x5
			/*
			Logger::getDefault()->log("P -> " + toString(P), DEBUG_MESSAGE);
			Logger::getDefault()->log("R -> " + toString(R), DEBUG_MESSAGE);
			Logger::getDefault()->log("H -> " + toString(H), DEBUG_MESSAGE);*/
			K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  			res.first = in.first + K * y;
  			res.second = (I - K * H) * P;
			//Logger::getDefault()->log("K -> " + toString(K), DEBUG_MESSAGE);
			//Logger::getDefault()->log("update " + toString(res.first) + ", " + toString(res.second), DEBUG_MESSAGE);
			return res;
		}
		double getRawAngle(){
			if(isForward){
				return start_angle - (hal->get_imu().get_rotation() / 180.0) * M_PI;
			}
			return start_angle - (hal->get_imu().get_rotation() / 180.0) * M_PI + M_PI;
		}
		bool isForward = true;
		double start_angle = 0;
		const int update_freq;
		const double update_delay = 1000.0 / update_freq;
		Eigen::Matrix4d F = Eigen::Matrix4d::Identity(), B = Eigen::Matrix4d::Zero(), I = Eigen::Matrix4d::Identity();
		const Eigen::Matrix4d Q{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 20.0, 0}, {0, 0, 0, 0.02}};
		Eigen::Matrix4d P{{10.0, 0, 0, 0}, {0, 10.0, 0, 0}, {0, 0, 0.01, 0}, {0, 0, 0, 0.05}}; //2.5 deg uncertainty and 10mm x and y uncertainty
		Eigen::Matrix<double, 5, 4> H; //5 rows, 4 cols
		Eigen::Matrix<double, 5, 5> R; //5 rows, 5 cols
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> prev_state;
		std::pair<Eigen::Vector4d, Eigen::Matrix4d> new_state;
		Eigen::Vector<double, 5> z;
		HAL *hal;
		Mixer *mixer;
		Mutex mutex;
};