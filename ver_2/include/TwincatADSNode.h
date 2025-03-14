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
    bool isOnline;
    AmsAddr Addr;
    // data type: double, bool[], double[], bool[], bool[].
    string adsVarNames[17] = {"MAIN.Axis_GoalPos", 
    "MAIN.bHomeSwitch[1]", "MAIN.bHomeSwitch[2]", "MAIN.bHomeSwitch[3]", "MAIN.bHomeSwitch[4]", 
    "MAIN.homeBusy[1]", "MAIN.homeBusy[2]", "MAIN.homeBusy[3]", "MAIN.homeBusy[4]",
    "MAIN.actPos[1]", "MAIN.actPos[2]", "MAIN.actPos[3]", "MAIN.actPos[4]",
    "MAIN.startMove[1]", "MAIN.startMove[2]", "MAIN.startMove[3]", "MAIN.startMove[4]"}; 
public:
    map<string, unsigned long> handlers;
    PAmsAddr pAddr;
    TwincatADSNode(bool isOnline=false);
    bool Connect(int port);
    void Disconnect();
    void WriteReq(string handle, double Data);
    void WriteReq(string handle, bool Data);
    void ReadReq(string handle, bool Data);
    void ReadReq(string handle, double Data);
    void ReadReq(string handle, int Data);
    void ReadReq(string handle, float Data);
};
#endif