#ifndef GripperController_H
#define GripperController_H

#include "ArduinoBLENode.h"
#include <string>
#include <map>

using namespace std;
class GripperController{
private:
    ArduinoBLENode node;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 20;
    string sendStr = "";
    bool isConnected;
public:
    GripperController(bool isOnline=true);

    void Connect(string portName);
    void Disconnect();
    bool IsConnected();

    void Open();
    void Close();
    void Rotate(int angle);
    void Calibrate();
};
#endif