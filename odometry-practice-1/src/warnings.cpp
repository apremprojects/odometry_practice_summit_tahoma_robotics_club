#include "logger.h"
#include "warnings.h"
Warning* Warning::getDefault(){
    static Warning *warnings = new Warning();
    return warnings;
}

Warning::Warning(): master(E_CONTROLLER_MASTER) {
    master.clear();
    warnings.fill(false);
}

void Warning::raise(const int id){
    mutex.take();
    warnings[id] = true;
    mutex.give();
}

void Warning::revoke(const int id){
    mutex.take();
    warnings[id] = false;
    mutex.give();
}

void Warning::update(){
    int line = 0;
    //clear screen more efficiently than master.clear()
    std::array<const char*, 3> disp_buf;
    for(int i = 0; i < 3; i++){
        master.clear_line(i);
        delay(100);
    }
    for(int warning = 0; warning < 100; warning++){
        {
            std::lock_guard<Mutex> lock(mutex);
            if(warnings[warning]){
                mutex.give();
                disp_buf[line] = toMessage(warning);
                Logger::getDefault()->log(toMessage(warning), DEBUG_MESSAGE);
                line++;
            }
        }
    }
    if(line > 3){
        disp_buf[2] = (std::string(disp_buf[2]) + "...").c_str();
    }
    for(int i = 0; i < line; i++){
        master.print(i, 0, disp_buf[i]);
        delay(100);
    }
    
}

const char* Warning::toMessage(const int id){
    switch(id){
        case 0:
            return "CTRLPT:FW";
        case 1:
            return "CTRLPT:REV";
        case 2:
            return "DPLY: CLMP";
        case 3:
            return "UNUSED";
        case 4:
            return "DPLY: INTKG";
        case 5:
            return "DPLY: EXHSTG";
        case 6:
            return "ELVTR: INTKG";
        case 7:
            return "ELVTR: EXHSTG";
    }
    return "INVDERRID";
}