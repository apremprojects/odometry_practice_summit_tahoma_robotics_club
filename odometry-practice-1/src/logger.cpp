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
    }
}
Logger::~Logger(){
    fclose(log_file);
}
void Logger::log(const std::string message, const int type){
    mutex.take(TIMEOUT_MAX);
    items.push(LogItem(message, millis()));
    std::string message_type = "INVALID_TYPE";
    switch(type){
        case FUNCTION_CALL:
            message_type = "FUNCTION_CALL";
        case POS_UPDATE:
            message_type = "POS_UPDATE";
        case DEBUG_MESSAGE:
            message_type = "DEBUG_MESSAGE";
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
    mutex.give();
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