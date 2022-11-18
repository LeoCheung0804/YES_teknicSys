#ifndef TwincatADSNode_H
#define TwincatADSNode_H
#include <string>
#include <vector>
#include <map>
#include <Windows.h>
#include "..\Dependencies\TcAdsDll\Include\TcAdsDef.h" // Define Ads Def-s before using API
#include "..\Dependencies\TcAdsDll\Include\TcAdsAPI.h"
using namespace std;
class TwincatADSNode{
private:
    AmsAddr Addr;
public:
    map<string, unsigned long> handlers = {
        {"MAIN.Axis_GoalPos", 0}, // double, 
        {"MAIN.startMove", 0}, // bool[]
        {"MAIN.actPos", 0}, // double[] 
        {"MAIN.bHomeSwitch", 0}, // bool[]
        {"MAIN.homeBusy", 0} // bool[]
    };
    
    PAmsAddr pAddr;
    TwincatADSNode();
    TwincatADSNode(int port);
    bool Connect(int port);
    void Disconnect();
};
#endif