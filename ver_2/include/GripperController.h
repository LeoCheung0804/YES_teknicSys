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
public:
    GripperController();
    GripperController(string portName, bool isOnline=true);
    void Open();
    void Close();
    void Rotate(float angle);
};
#endif