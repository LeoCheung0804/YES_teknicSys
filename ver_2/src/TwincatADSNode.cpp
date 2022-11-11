#include <iostream>
#include <string>
#include <vector>
#include <Windows.h>
// #include "ABBgetVar.h"
#include "..\Dependencies\TcAdsDll\Include\TcAdsDef.h" // Define Ads Def-s before using API
#include "..\Dependencies\TcAdsDll\Include\TcAdsAPI.h"
// #include "Dependencies\DynamixelSDK-3.7.31\include\dynamixel_sdk\dynamixel_sdk.h"
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
    TwincatADSNode(int port){ 
        this->port = port; 
    }
    bool Connect(){
        long nErr, nPort;
        unsigned long lHdlVar;
        bool TRUE_FLAG = true;

        cout << "Connecting to Twincat ADS Nodes and get handlers." << endl;
        // Open communication port on the ADS router
        nPort = AdsPortOpen();
        nErr = AdsGetLocalAddress(pAddr);
        if (nErr){ cout << "Error: AdsGetLocalAddress: " << nErr << '\n'; return false; }
        pAddr->port = this->port;

        // Enable power for motors, start state machine
        // Get handle of the request
        nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, sizeof("MAIN.power"), "MAIN.power");
        if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << '\n'; return false; }
        // Use the handle to write data
        nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_VALBYHND,lHdlVar, sizeof(TRUE_FLAG), &TRUE_FLAG);
        if (nErr){ cout << "Error: AdsSyncWriteReq: " << nErr << '\n'; return false; }
        else { cout << "Linear rail motors enabled.\n"; }
        // Release the handle
        nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_RELEASEHND,0,sizeof(lHdlVar),&lHdlVar);

        // Create list of handler by name
        for(int i=0; i < *(&adsVarNames+1)-adsVarNames; i++){
            nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, adsVarNames[i].length(), &adsVarNames[i][0]);
            if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << ", in creating handler for " << adsVarNames[i] << "\n"; return false; }
            hdlList.push_back(lHdlVar);
        }
        cout << "Completed creating handler list of " << hdlList.size() << endl;
        return true;
    }
};