#ifndef CableController_H
#define CableController_H

#include "TeknicNode.h"

using namespace std;
using namespace sFnd;
class CableController{
private:
    TeknicNode node;
    int cableNumber;
    vector<INode*> nodeList;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 20;
public:
    CableController();
    CableController(int cableNumber, bool isOnline=true);
    void TightenCableByIndex(int index, float targetTrq);
    void TightenAllCable(float targetTrq);
    void HomeAllCableMotors();
    void MoveSingleMotorCmd(int index, int32_t cmd);
    void MoveAllMotorCmd(vector<int32_t> cmdList);
    bool IsMoveFinished();    
    void CalibrationMotor(int index, int32_t currentCmdPos);
    double GetMotorPosMeasured(int index);
};
#endif