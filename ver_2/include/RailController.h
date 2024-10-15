#ifndef RailController_H
#define RailController_H

#include "TwincatADSNode.h"
#include "vector"

using namespace std;
class RailController{
private:
    TwincatADSNode motorNode;
    int railNumber;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 50;
    string sendStr;
    bool *bArry;
    int workingMotor = 0;
    bool isConnected;
    bool useRail;
public:
    RailController(bool isOnline=false, bool useRail=false);
    void Connect(int motorPortNumber, int railNumber);
    void Disconnect();
    bool IsConnected();
    void CalibrationMotor(int index, double currentCmdPos);
    void SelectWorkingMotor(int index);
    void MoveSelectedMotorCmd(double cmd, bool absolute=true);
    vector<int> GetMotorPosMeasured();
};
#endif