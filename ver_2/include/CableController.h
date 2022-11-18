#ifndef CableController_H
#define CableController_H

#include "TeknicNode.h"
#include "ArduinoBLENode.h"

using namespace std;
using namespace sFnd;
class CableController{
private:
    TeknicNode motorNode;
    ArduinoBLENode breakNode;
    int cableNumber;
    int brakeNumber;
    vector<INode*> nodeList;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 20;
    string sendStr;
public:
    CableController();
    CableController(int cableNumber, int brakeNumber, string brakePortName, bool isOnline=true);
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
    void OpenBreak(int index);
    void CloseBreak(int index);
};
#endif