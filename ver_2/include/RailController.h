#ifndef RailController_H
#define RailController_H

#include "TwincatADSNode.h"
#include "ArduinoBLENode.h"
#include "vector"

using namespace std;
class RailController{
private:
    TwincatADSNode motorNode;
    ArduinoBLENode breakNode;
    int railNumber;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 20;
    string sendStr;
    bool *bArry;
    int workingMotor = 0;
    bool isConnected;
public:
    RailController(bool isOnline=true);
    void Connect(int motorPortNumber, int railNumber, string breakPortName);
    void Disconnect();
    bool IsConnected();
    void OpenBreak(int index);
    void CloseBreak(int index);
    void CalibrationMotor(int index, int32_t currentCmdPos);
    void HomeAllMotors();
    void SelectWorkingMotor(int index);
    void MoveSelectedMotorCmd(int32_t cmd, bool absolute=true);
    vector<int> GetMotorPosMeasured();
};
#endif