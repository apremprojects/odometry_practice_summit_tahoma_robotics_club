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
#include <fstream>
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
        void log(const std::string input_message, const int type);
		void start(){
			    if(pros::usd::is_installed()){
				log("SD Card installed", DEBUG_MESSAGE);
				log_file = fopen(filename, "a");
				setbuf(log_file, NULL);
				if (log_file == NULL) {
					perror("SD Card Error -> ");
					is_file_available = false;
				}
				else {
					printf("SD Card File Available...");
					is_file_available = true;
				}
			}
			else{
				log("SD Card not installed", DEBUG_MESSAGE);
				is_file_available = false;
			}
		}
		void restart(){
			std::lock_guard<Mutex> lock(mutex);
			if(is_file_available){
				fclose(log_file);
				//is_file_available = false;
				FILE *tmp = fopen(filename, "a");
				log_file = tmp;
			}
		}
		//return true the SD card available, else, returns false
		bool isFileAvailable(){
			return is_file_available;
		}
		//returns logfile reference incase I want to do some extremely highthroughput read/write OR read earlier logs
		FILE *getFilePtr(){
			return log_file;
		}
        static Logger* getDefault();
		~Logger();
	private:
		Logger();
		int last_close_time = 0;
		const char* filename = "/usd/log.log";
 		std::string toTimestamp(const int timestamp);
		Mutex mutex;
		std::queue<LogItem> items;
		bool is_file_available = false;
		int logct = 0;
		FILE *log_file;
 };