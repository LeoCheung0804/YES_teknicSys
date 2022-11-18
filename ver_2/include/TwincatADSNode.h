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
    //// Initialize linear rails through Twincat ADS
    AmsAddr       Addr;
    double actPos[4]{};
public:
    string adsVarNames[5] = {"MAIN.Axis_GoalPos", "MAIN.startMove", "MAIN.actPos", "MAIN.bHomeSwitch", "MAIN.homeBusy"}; // data type: double, bool[], double[], bool[], bool[].
    vector<unsigned long> hdlList; // create list for easy use of handlers, listed in string adsVarNames[]
    map<string, unsigned long> handles = {
        {"MAIN.Axis_GoalPos", 0},
        {"MAIN.startMove", 0},
        {"MAIN.actPos", 0},
        {"MAIN.bHomeSwitch", 0},
        {"MAIN.homeBusy", 0}
    };
    PAmsAddr      pAddr = &Addr;
    TwincatADSNode();
    TwincatADSNode(int port);
    bool Connect(int port);
    void Disconnect();
};
#endif