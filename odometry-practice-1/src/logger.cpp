#include "logger.h"
#include <unistd.h>
#include <iostream>
#include <cstring>

Logger::Logger() {
    if(pros::usd::is_installed()){
        log("SD Card installed", DEBUG_MESSAGE);
        log_file = fopen(filename, "w");
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

Logger::~Logger(){
    fclose(log_file);
}

void Logger::log(const std::string input_message, const int type){
    std::lock_guard<Mutex> lock(mutex);
    items.push(LogItem(input_message, millis()));
    std::string message_type;
    if(type == FUNCTION_CALL){
        message_type = "FUNCTION_CALL";
    }
    else if(type == DEBUG_MESSAGE){
        message_type = "DEBUG_MESSAGE";
    }
    else if(type == POS_UPDATE){
        message_type = "POS_UPDATE";
    }
    else if(type == TARGET_POS_UPDATE){
        message_type = "TARGET_POS_UPDATE";
    }
    else if(type == ANGLE_UPDATE){
        message_type = "ANGLE_UPDATE";
    }
    else if(type == TARGET_ANGLE_UPDATE){
        message_type = "TARGET_ANGLE_UPDATE";
    }
    else if(type == VELOCITY_UPDATE){
        message_type = "VELOCITY_UPDATE";
    }
    else if(type == TARGET_VELOCITY_UPDATE){
        message_type = "TARGET_VELOCITY_UPDATE";
    }
    else if(type == THROTTLE_UPDATE){
        message_type = "THROTTLE_UPDATE";
    }
    else if(type == YAW_UPDATE){
        message_type = "YAW_UPDATE";
    }
    else if(type == WARNING){
        message_type = "WARNING";
    }
    else if(type == DEPLOY_UPDATE){
        message_type = "DEPLOY_UPDATE";
    }
    else if(type == INTAKE){
        message_type = "INTAKE";
    }
    else if(type == PID_STATUS) {
        message_type = "PID_STATUS";
    }
    else if(type == CONTROL_POINT){
        message_type = "CONTROL_POINT";
    }
    else if(type == MODE_SWITCH){
        message_type = "MODE_SWITCH";
    }
    while(!items.empty()){
        std::string message = toTimestamp(items.front().timestamp) + " " + message_type + " " + items.front().message + "\n";
        if(!is_file_available){
            std::cout << message;
        }
        else{
            //std::cout << message;
            const char* m = message.c_str();
            fwrite(m, sizeof(char), strlen(m), log_file);
            fflush(log_file);
        }
        //then print log to screen
        items.pop();
    }
}

std::string Logger::toTimestamp(const int timestamp){
    int hh = (timestamp / 3600000) % 24; //hour
    int mm = (timestamp / 60000) % 60;
    int ss = (timestamp / 1000) % 60;
    int ms = timestamp % 1000;
    return std::to_string(hh) + ":" + std::to_string(mm) + ":" + std::to_string(ss) + ":" + std::to_string(ms);
}

Logger* Logger::getDefault(){
    static Logger *logger = new Logger();
    return logger;
}