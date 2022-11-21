#ifndef Logger_H
#define Logger_H

#include <string>
#include <iostream>
#include <fstream>

using namespace std;

class Logger{
private:
    ofstream file;
    string filename;
    string posLabel[6]{"x", "y", "z", "yaw", "pitch", "roll"}; 
public:
    Logger();
    bool OpenFile(string filename);
    void CloseFile();
    void Log(string msg);
    void LogError(string msg);
    void LogWarning(string msg);
    void LogInfo(string msg);
    void LogPos(double eePos[], double railPos[]);
};

#endif