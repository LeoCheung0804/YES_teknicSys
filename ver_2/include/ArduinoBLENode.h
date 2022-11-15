#ifndef ArduinoBLENode_H
#define ArduinoBLENode_H

#include <Windows.h>
#include <string>

using namespace std;
class ArduinoBLENode{
private:
    string ComPortName;
    unsigned char Ard_char[8] = {'(','o',',',' ',' ',' ',' ',')'};
    HANDLE hComm;
    
    // set communication port
    bool SetSerialParams(HANDLE hComm);
public:
    ArduinoBLENode();
    ArduinoBLENode(string portName);

    // Connect Node
    bool Connect();
};
#endif