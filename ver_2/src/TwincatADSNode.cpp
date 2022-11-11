
#include <vector>
#include <iostream>
#include <string>
#include "..\Dependencies\TcAdsDll\Include\TcAdsDef.h" // Define Ads Def-s before using API
#include "..\Dependencies\TcAdsDll\Include\TcAdsAPI.h"

using namespace std;
const int TwinCat_NUM = 4; // No. of axes listed in TwinCat programme
class TwincatADSNode{
private:
    int port = 851;
    //// Initialize linear rails through Twincat ADS
    AmsAddr       Addr;
    PAmsAddr      pAddr = &Addr;
    long nErr{0};
    double actPos[TwinCat_NUM]{};
    vector<unsigned long> hdlList; // create list for easy use of handlers, listed in string adsVarNames[]
public:
    TwincatADSNode(){};
    TwincatADSNode(int port){this->port = port};
    void Connect(){
        long nErr, nPort;
        unsigned long lHdlVar;
        bool TRUE_FLAG = true;

        // Open communication port on the ADS router
        nPort = AdsPortOpen();
        nErr = AdsGetLocalAddress(pAddr);
        if (nErr){ cout << "Error: AdsGetLocalAddress: " << nErr << '\n'; return nErr; }
        pAddr->port = this->port;

        // Enable power for motors, start state machine
        // Get handle of the request
        nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, sizeof("MAIN.power"), "MAIN.power");
        if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << '\n'; return nErr; }
        // Use the handle to write data
        nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_VALBYHND,lHdlVar, sizeof(TRUE_FLAG), &TRUE_FLAG);
        if (nErr){ cout << "Error: AdsSyncWriteReq: " << nErr << '\n'; return nErr; }
        else { cout << "Linear rail motors enabled.\n"; }
        // Release the handle
        nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_RELEASEHND,0,sizeof(lHdlVar),&lHdlVar);

        // Create list of handler by name
        string adsVarNames[] = {"MAIN.Axis_GoalPos", "MAIN.startMove", "MAIN.actPos", "MAIN.bHomeSwitch", "MAIN.homeBusy"}; // data type: double, bool[], double[], bool[], bool[].
        for(int i=0; i < *(&adsVarNames+1)-adsVarNames; i++){
            nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, adsVarNames[i].length(), &adsVarNames[i][0]);
            if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << ", in creating handler for " << adsVarNames[i] << "\n"; return nErr; }
            hdlList.push_back(lHdlVar);
        }
        cout << "Completed creating handler list of " << hdlList.size() << endl;
        return 0;
    }
}