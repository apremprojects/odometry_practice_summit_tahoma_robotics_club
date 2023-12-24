#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#define _USE_MATH_DEFINES
#define FUNCTION_CALL 0
#define DEBUG_MESSAGE 1
#define POS_UPDATE 2
#define TARGET_POS_UPDATE 3
#define ANGLE_UPDATE 4
#define TARGET_ANGLE_UPDATE 5
#define VELOCITY_UPDATE 6
#define TARGET_VELOCITY_UPDATE 7
#define THROTTLE_UPDATE 8
#define YAW_UPDATE 9
#define WARNING 10
#define DEPLOY_UPDATE 11
#define INTAKE 12
#define PID_STATUS 13
#define CONTROL_POINT 14
#define MODE_SWITCH 15

#include "api.h"
#include <mutex>
#include <cmath>
#include <string>
#include <queue>
using namespace pros;
using namespace pros::literals;

class LogItem{
	public:
		LogItem(const std::string _message, const int _timestamp): message(_message), timestamp(_timestamp){};
		std::string message;
		int timestamp;
};

class Logger{
    public:
		//logs a message to terminal AND/OR file.
        void log(const std::string message, const int type);
		//return true the SD card available, else, returns false
		bool isFileAvailable(){
			return is_file_available;
		}
		//returns logfile pointer incase I want to do some extremely highthroughput read/write OR read earlier logs
		FILE *getFilePtr(){
			return log_file;
		}
        static Logger* getDefault();
	private:
       Logger();
	   ~Logger();
 		std::string toTimestamp(const int timestamp);
		Mutex mutex;
		std::queue<LogItem> items;
		bool is_file_available;
        FILE *log_file;
 };