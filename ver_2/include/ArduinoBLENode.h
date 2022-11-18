#ifndef ArduinoBLENode_H
#define ArduinoBLENode_H

#include <Windows.h>
#include <string>

using namespace std;
class ArduinoBLENode{
private:
    HANDLE hComm;
    int readTimeout=1000;
    
    // set communication port
    bool SetSerialParams();
public:
    ArduinoBLENode();

    bool Connect(string portName);  
    void Disconnect();

    string Read();
    string Send(string Ard_char);
};
#endif