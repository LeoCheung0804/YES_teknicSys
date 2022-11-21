#ifndef RailController_H
#define RailController_H

#include "TwincatADSNode.h"
#include "ArduinoBLENode.h"
#include "vector"

using namespace std;
class RailController{
private:
    TwincatADSNode motorNode;
    ArduinoBLENode brakeNode;
    int railNumber;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 20;
    string sendStr;
    bool *bArry;
    bool* brakeOnFlags;
    int workingMotor = 0;
    bool isConnected;
    bool useMotor;
    bool useBraker;
public:
    RailController(bool isOnline=true);
    void Connect(int motorPortNumber, int railNumber, string brakePortName, bool useMotor, bool useBraker);
    void Disconnect();
    bool IsConnected();
    void OpenBrake(int index);
    void CloseBrake(int index);
    void OpenAllBrake();
    void CloseAllBrake();
    void CalibrationMotor(int index, int32_t currentCmdPos);
    void HomeAllMotors();
    void SelectWorkingMotor(int index);
    void MoveSelectedMotorCmd(int32_t cmd, bool absolute=true);
    vector<int> GetMotorPosMeasured();
};
#endif