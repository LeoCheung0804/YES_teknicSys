#ifndef CableController_H
#define CableController_H

#include "TeknicNode.h"
#include "ArduinoBLENode.h"

using namespace std;
using namespace sFnd;
class CableController{
private:
    TeknicNode motorNode;
    ArduinoBLENode brakeNode;
    int cableNumber;
    int brakeNumber;
    vector<INode*> nodeList;
    bool isOnline;
    bool isConnected;
    bool isMoveFinished;
    bool* brakeOnFlags;
    int MILLIS_TO_NEXT_FRAME = 20;
    string sendStr;
    bool useMotor;
    bool useBraker;
public:
    CableController(bool isOnline=true);
    void Connect(int cableNumber, int brakeNumber, string brakePortName, bool useMotor, bool useBraker);
    void Disconnect();
    bool IsConnected();
    void TightenCableByIndex(int index, float targetTrq);
    void TightenAllCable(float targetTrq);
    void HomeAllMotors();
    void MoveSingleMotorCmd(int index, int32_t cmd, bool absolute=true);
    void MoveAllMotorCmd(vector<int32_t> cmdList, bool absolute=true);
    void MoveAllMotorCmd(int32_t cmd, bool absolute=true);
    bool IsMoveFinished();    
    void CalibrationMotor(int index, int32_t currentCmdPos);
    double GetMotorPosMeasured(int index);
    double GetMotorTorqueMeasured(int index);
    void SavePosToFile(string filename);
    void OpenBrake(int index);
    void CloseBrake(int index);
    void OpenAllBrake();
    void CloseAllBrake();
};
#endif