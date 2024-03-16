#include "dsl.h"

DSLInterpreter::DSLInterpreter(const std::string &_code, Robot *_robot): code(_code), robot(_robot) {
    std::transform(code.begin(), code.end(), code.begin(), ::toupper);
    for(int i = 0; i < code.size(); i++){
        if(code[i] == '\n' && i + 1 < code.size()){
            line_starts.push_back(i + 1);
        }
    }
}

void DSLInterpreter::run(){
    while(line < line_starts.size()){ //while line < end of file
        //command = line_starts[line] to line_starts[line+1]
        std::string l = code.substr(line_starts[line], line_starts[line+1]);
        std::istringstream ss(l);
        std::string command;
        ss >> command;
        int size = std::distance(std::istream_iterator<std::string>(ss), std::istream_iterator<std::string>());
        if (command == "MILLIS") {
            if(size == 2){
                int var;
                ss >> var;
                nums[var] = millis();
            }
            else{
                //SYNTAX_ERROR
            }
        }
        else if(command == "GOTO_POS"){
            if(size == 5){
                std::string x, y, v, d;
                int id;
                ss >> x >> y >> v >> d >> id;
                double xi = 0, yi = 0, vi = 0;
                bool db = false;

                if(type(x) == TYPE::NUM){
                    xi = nums[stoi(x)];
                }
                else if(type(x) == TYPE::CONST){
                    xi = stoi(x);
                }
                else{
                    //SYNTAX ERROR
                }

                if(type(y) == TYPE::NUM){
                    yi = nums[stoi(y)];
                }
                else if(type(y) == TYPE::CONST){
                    yi = stoi(y);
                }
                else{
                    //SYNTAX ERROR
                }

                if(type(v) == TYPE::NUM){
                    vi = nums[stoi(v)];
                }
                else if(type(v) == TYPE::CONST){
                    vi = stoi(v);
                }
                else{
                    //SYNTAX ERROR
                }

                if(type(d) == TYPE::BOOL){
                    db = bools[stoi(d)];
                }
                else if(type(v) == TYPE::CONST){
                    db = stoi(d);
                }
                else{
                    //SYNTAX ERROR
                }
                statuses[id] = robot->goto_pos(vi, 0, xi, yi, db);
                wps[id] = std::pair<int, int>(xi, yi);
            }
            else{
                //SYNTAX_ERROR
            }
        }
        else if(command == "DISTANCE"){
            if(size == 3){
                std::string id, str;
                ss >> id >> str;
                int idi = 0, stri = 0;
                if(type(id) == TYPE::CONST){
                    idi = stoi(id);
                }
                else if(type(id) == TYPE::NUM){
                    idi = nums[stoi(id)];
                }
                else{
                    //SYNTAX ERROR
                }
                
                if(type(str) == TYPE::NUM){
                    stri = stoi(id);
                }
                else{
                    //SYNTAX ERROR
                }
                nums[stri] = robot->get_distance(wps[idi].first, wps[idi].second, robot->getX(), robot->getY());
            }
        }
        else if(command == "IS_DONE"){
            if(size == 3){
                std::string id, str;
                ss >> id >> str;
                int idi = 0, stri = 0;
                if(type(id) == TYPE::NUM){
                    idi = nums[stoi(id)];
                }
                else if(type(id) == TYPE::CONST){
                    idi = stoi(id);
                }
                else{
                    //SYNTAX ERROR
                }

                if(type(str) == TYPE::BOOL){
                    stri = stoi(str);
                }
                else{
                    //SYNTAX ERROR
                }

                bools[stri] = statuses[idi]->done;
            }
        }
        else if(command == "JMP"){
            if(size == 3){
                std::string new_line, b_adr;
                ss >> new_line >> b_adr;
                int new_linei = 0, b_adri = 0;

                if(type(new_line) == TYPE::NUM){
                    new_linei = nums[stoi(new_line)];
                }
                else if(type(new_line) == TYPE::CONST) {
                    new_linei = stoi(new_line);
                }

                if(type(b_adr) == TYPE::BOOL){
                    b_adri = stoi(b_adr);
                }
                else{
                    //SYNTAX ERROR
                }

                if(bools[b_adri]){
                    line = new_linei;
                }
            }
        }
    }
}