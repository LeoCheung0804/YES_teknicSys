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
    // data type: double, bool[], double[], bool[], bool[].
    string adsVarNames[5] = {"MAIN.Axis_GoalPos", "MAIN.startMove", "MAIN.actPos", "MAIN.bHomeSwitch", "MAIN.homeBusy"}; 
public:
    map<string, unsigned long> handlers;
    PAmsAddr pAddr;
    TwincatADSNode();
    TwincatADSNode(int port);
    bool Connect(int port);
    void Disconnect();
};
#endif