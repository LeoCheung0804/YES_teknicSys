#ifndef GripperController_H
#define GripperController_H

#include "COMPortNode.h"
#include <string>
#include <map>

using namespace std;
class GripperController{
private:
    COMPortNode node;
    bool isOnline;
    bool useGripper;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 50;
    string sendStr = "";
    bool isConnected;
public:
    GripperController(bool isOnline=false, bool useGripper=false);

    void Connect(string portName);
    void Disconnect();
    bool IsConnected();

    void Open();
    void Close();
    void Rotate(int angle);
    void Release();
    void SetCalibrateStartAngle(int angle);
    void SetCalibrateEndAngle(int angle);
    void Calibrate();
};
#endif