#ifndef BrakeController_H
#define BrakeController_H

#include "COMPortNode.h"
#include <string>
#include <map>

using namespace std;
class BrakeController{
private:
    COMPortNode node;
    bool isOnline;
    bool useRailBrake;
    bool useCableBrake;
    
    string sendStr = "";
    bool isConnected;


    int railBrakeNum;
    int cableBrakeNum;

    bool* railBrakeFlag;
    bool* cableBrakeFlag;
    
public:
    BrakeController(bool isOnline=true);

    void Connect(string portName);
    
    void Disconnect();
    bool IsConnected();
    
    void UseRailBrake(int val);
    void UseCableBrake(int val);

    void OpenRailBrakeByIndex(int index);
    void CloseRailBrakeByIndex(int index);

    void OpenAllRailBrake();
    void CloseAllRailBrake();
        
    void OpenCableBrakeByIndex(int index);
    void CloseCableBrakeByIndex(int index);

    void OpenAllCableBrake();
    void CloseAllCableBrake();

};
#endif