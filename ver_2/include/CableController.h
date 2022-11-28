#ifndef CableController_H
#define CableController_H

#include "TeknicNode.h"
#include "ArduinoBLENode.h"
#include "Logger.h"

using namespace std;
using namespace sFnd;
class CableController{
private:
    TeknicNode motorNode;
    ArduinoBLENode brakeNode;
    int cableNumber;
    int brakeNumber;
    float absTrqLmt;
    vector<INode*> nodeList;
    bool isOnline;
    bool isConnected;
    bool isMoveFinished;
    bool* brakeOnFlags;
    int MILLIS_TO_NEXT_FRAME = 50;
    string sendStr;
    bool useMotor;
    bool useBraker;
    bool eStop;
    Logger logger;
public:
    CableController(bool isOnline=true);
    void Connect(int cableNumber, int brakeNumber, string brakePortName, bool useMotor, bool useBraker);
    void Disconnect();
    bool IsConnected();
    void SetCableTrqByIndex(int index, float targetTrq, float tolerance);
    void SetCableTrq(float targetTrq, float tolerance);
    bool MoveSingleMotorCmd(int index, int32_t cmd, bool absolute=true);
    bool MoveAllMotorCmd(vector<int32_t> cmdList, bool absolute=true);
    bool MoveAllMotorCmd(int32_t cmd, bool absolute=true);
    void CalibrationMotor(int index, int32_t currentCmdPos);
    double GetMotorPosMeasured(int index);
    double GetMotorTorqueMeasured(int index);
    void StopAllMotor();
    void OpenBrake(int index);
    void CloseBrake(int index);
    void OpenAllBrake();
    void CloseAllBrake();
    void SetTrqLmt(float lmt);
    void ClearAlert();
};
#endif