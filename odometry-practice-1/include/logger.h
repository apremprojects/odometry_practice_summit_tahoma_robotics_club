#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#define _USE_MATH_DEFINES
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
        void log(const std::string message);
        static Logger* getDefault();
	private:
       Logger();
 		std::string to_timestamp(const int timestamp);
		Mutex mutex;
		std::queue<LogItem> items;
        FILE *revision;
        FILE *output;
 };