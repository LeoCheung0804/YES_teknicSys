#ifndef RailController_H
#define RailController_H

#include "TwincatADSNode.h"
#include "ArduinoBLENode.h"

using namespace std;
class RailController{
private:
    TwincatADSNode motorNode;
    ArduinoBLENode breakNode;
    int railNumber;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 20;
public:
    RailController();
    RailController(int motorPortNumber, string breakPortName, int railNumber, bool isOnline=true);
};
#endif