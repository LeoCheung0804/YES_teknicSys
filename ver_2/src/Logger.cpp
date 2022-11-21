#include "..\include\Logger.h"

Logger::Logger(){}

bool Logger::OpenFile(string filename){
    this->filename = filename;
    this->file = ofstream(filename, ios::app);
    if(this->file.good()){
        cout << "Opened log file: " << filename << " success." << endl;
        return true;
    }else{
        cout << "Opened log file: " << filename << "failed." << endl;
        return false;
    }
}

void Logger::CloseFile(){
    this->file.close();
    cout << "Closed log file: " << this->filename << endl;
}

void Logger::Log(string msg){
    tm *fn; time_t now = time(0); fn = localtime(&now);
    this->file << "[" << fn->tm_year << "/" << fn->tm_mon +1 << "/" << fn->tm_mday << ": " << fn->tm_hour << ":" << fn->tm_min << ":" << fn->tm_sec << "]" ;
    this->file << msg;
    this->file <<endl;
}

void Logger::LogError(string msg){
    this->Log("[Error]" + msg);
}

void Logger::LogWarning(string msg){
    this->Log("[Warning]" + msg);
}

void Logger::LogInfo(string msg){
    this->Log("[Info]" + msg);
}

void Logger::LogPos(double eePos[], double railPos[]){
    string msg = "[Pos]";
    for(int i = 0; i < 6; i++){
        msg += this->posLabel[i];
        msg += ": " + to_string(eePos[i]) + ", ";
    }
    
    for(int i = 0; i < 4; i++){
        msg += "cable " + to_string(i);
        msg += " : " + to_string(eePos[i]);
        if(i < 3) msg += ", ";
    }
    this->Log(msg);
}

