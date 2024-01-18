#pragma once
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
#define _USE_MATH_DEFINES
#include "api.h"
#include <mutex>
#include <cmath>
#include <array>
using namespace pros;
using namespace pros::literals;

class Warning{
    public:
		//logs a message to terminal AND/OR file.
        void raise(const int id);
        void revoke(const int id);
        void update();
		//return true the SD card available, else, returns false
        static Warning* getDefault();
	private:
        Warning();
 	    const char* toMessage(const int id);
        pros::Controller master;
		Mutex mutex;
        std::array<bool, 100> warnings;
 };