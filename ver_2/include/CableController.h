#ifndef CableController_H
#define CableController_H

#include "TeknicNode.h"
#include "Logger.h"

using namespace std;
using namespace sFnd;
class CableController{
private:
    TeknicNode motorNode;
    int cableNumber;
    float absTrqLmt;
    vector<INode*> nodeList;
    bool isOnline;
    bool useCable;
    bool isConnected;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 50;
    string sendStr;
    bool eStop;
    Logger logger;
public:
    CableController(bool isOnline=false, bool useCable=false);
    void Connect(int cableNumber);
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
    void SetTrqLmt(float lmt);
    void ClearAlert();
};
#endif