#include "logger.h"
#include <unistd.h>
#include <iostream>

Logger::Logger(){
    log_file = fopen("/usd/log.log", "w+");
    log("Logger initialized...", DEBUG_MESSAGE);
    if(log_file != NULL){
        log("FAT32 SDCard present...", DEBUG_MESSAGE);
    }
    else{
        log("Insert FAT32 SDCard...", DEBUG_MESSAGE);
        //exit(1);
        is_file_available = false;
    }
}
Logger::~Logger(){
    fclose(log_file);
}
void Logger::log(const std::string message, const int type){
    std::lock_guard<Mutex> lock(mutex);
    items.push(LogItem(message, millis()));
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
        std::cout << message;
        if(log_file != NULL){
            std::cout << "Logging to file\n";
            const char *cm = message.c_str();
            fputs(cm, log_file);
        }
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