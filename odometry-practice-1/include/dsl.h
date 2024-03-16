#pragma once
#include "api.h"
#include "logger.h"
#include "robot.h"
#include <cmath>
#include <string>
#include <queue>
#include <sstream>
using namespace pros;
using namespace pros::literals;

enum class TYPE {
    NUM,
    BOOL,
    CONST
};

class DSLInterpreter{
    public:
        DSLInterpreter(const std::string &_code, Robot *_robot);
        void run();
    private:
        TYPE type(const std::string &s){
            if(s[0] == '#'){
                return TYPE::NUM;
            }
            else if(s[0] == '$'){
                return TYPE::BOOL;
            }
            return TYPE::CONST;
        }
        int line = 0;
        Robot *robot;
        std::string code;
        std::vector<int> line_starts;
        std::array<double, 1000> nums;
        std::array<bool, 1000> bools;
        std::array<std::pair<double, double>, 1000> wps;
        std::array<Status*, 1000> statuses;
};
/*
Language is asynchronous

#1 - #inf are variables
$1 - $1 are booleans

MILLIS #1

(X, Y, and ID are constants)
GOTO_POS X Y V D ID
GOTO_POS #1 #2 V D ID

DISTANCE ID #1
IS_DONE ID $1

JMP A $1
JMP #1 $2

MULTIPLY #1 #2 #3
MULTIPLY A B #1

ADD #1 #2 #3
ADD A B #1

SUBTRACT #1 #2 #3
SUBTRACT A B #1

DIVIDE #1 #2 #3
DIVIDE A B #1

SIN A #1
SIN #2 #2

COS A #1
COS #2 #2

ROUND 7.6 #1
ROUND #1 #2

TRUNCATE 7.6 #1
TRUNCATE #1 #2

STORE #1 VAL (VAL is a constant)
STORE #1 #2

LOG 7
LOG #1

INTAKE STOP
INTAKE INTAKE
INTAKE REVERSE

WING RIGHT DEPLOY
WING RIGHT RETRACT
WING LEFT DEPLOY
WING LEFT RETRACT
*/