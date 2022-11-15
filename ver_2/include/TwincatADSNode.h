#ifndef TwincatADSNode_H
#define TwincatADSNode_H
#include <string>
#include <vector>
#include <Windows.h>
#include "..\Dependencies\TcAdsDll\Include\TcAdsDef.h" // Define Ads Def-s before using API
#include "..\Dependencies\TcAdsDll\Include\TcAdsAPI.h"
using namespace std;
class TwincatADSNode{
private:
    int port;
    //// Initialize linear rails through Twincat ADS
    AmsAddr       Addr;
    PAmsAddr      pAddr = &Addr;
    double actPos[4]{};
    string adsVarNames[5] = {"MAIN.Axis_GoalPos", "MAIN.startMove", "MAIN.actPos", "MAIN.bHomeSwitch", "MAIN.homeBusy"}; // data type: double, bool[], double[], bool[], bool[].
    vector<unsigned long> hdlList; // create list for easy use of handlers, listed in string adsVarNames[]
public:
    TwincatADSNode();
    TwincatADSNode(int port);
    bool Connect();
};
#endif