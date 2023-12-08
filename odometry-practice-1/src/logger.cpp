#include "logger.h"
#include <iostream>

Logger::Logger(){
    log("Logger initialized...");
    revision = fopen("/usd/revision.log", "w+");
}
void Logger::log(const std::string message){
    mutex.take(TIMEOUT_MAX);
    items.push(LogItem(message, millis()));
    while(!items.empty()){
        std::cout << to_timestamp(items.front().timestamp) << " -> " << items.front().message << "\n";
        items.pop();
    }
    mutex.give();
}
std::string Logger::to_timestamp(const int timestamp){
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