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
public:
    CableController();
    CableController(int cableNumber, bool isOnline=true);
    void TightenCableByIndex(int index, float targetTrq);
    void TightenAllCable(float targetTrq);
    void HomeAllCableMotors();
    void MoveToLength(float cableLength[]);
    bool IsMoveFinished();    
};
#endif