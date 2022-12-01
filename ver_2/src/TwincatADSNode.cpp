#include "..\include\TwincatADSNode.h"
#include <iostream>
TwincatADSNode::TwincatADSNode(bool isOnline){ this->isOnline = isOnline; }

bool TwincatADSNode::Connect(int port){
    if(!this->isOnline) return true;
    long nErr, nPort;
    unsigned long lHdlVar;
    bool TRUE_FLAG = true;

    cout << "Connecting to Twincat ADS Nodes." << endl;
    // Open communication port on the ADS router
    nPort = AdsPortOpen();
    nErr = AdsGetLocalAddress(pAddr);
    if (nErr){ cout << "Error: AdsGetLocalAddress: " << nErr << '\n'; return false; }
    pAddr->port = port;

    // Enable power for motors, start state machine
    // Get handle of the request
    nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, sizeof("MAIN.power"), "MAIN.power");
    if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << '\n'; return false; }
    // Use the handle to write data
    nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_VALBYHND, lHdlVar, sizeof(TRUE_FLAG), &TRUE_FLAG);
    if (nErr){ cout << "Error: AdsSyncWriteReq: " << nErr << '\n'; return false; }
    else { cout << "Linear rail motors enabled.\n"; }
    // Release the handle
    nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_RELEASEHND, 0,sizeof(lHdlVar),&lHdlVar);

    // Create list of handler by name

    for(int i=0; i < *(&adsVarNames+1)-adsVarNames; i++){
        nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, adsVarNames[i].length(), &adsVarNames[i][0]);
        if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << ", in creating handler for " << adsVarNames[i] << "\n"; return nErr; }
        handlers[adsVarNames[i]] = lHdlVar;
    }
    cout << "Completed creating handler list of " << handlers.size() << endl;
    return true;
}

void TwincatADSNode::Disconnect(){
    if(!this->isOnline);
    long nErr;
    // Close ADS communication port, disable power for motors
    long lHdlVar; 
    bool FALSE_FLAG = false;
    nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, sizeof("MAIN.power"), "MAIN.power");
    if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << '\n'; }
    nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_VALBYHND,lHdlVar, sizeof(FALSE_FLAG), &FALSE_FLAG);
    if (nErr){ cout << "Error: AdsSyncWriteReq: " << nErr << '\n'; }
    else { cout << "Linear rail motors disabled.\n"; }
    nErr = AdsSyncWriteReq(pAddr,ADSIGRP_SYM_RELEASEHND,0,sizeof(lHdlVar),&lHdlVar);
    if ( AdsPortClose() ){ cout << "Error: AdsPortClose: " << nErr << '\n'; }
}

void TwincatADSNode::WriteReq(string handle, int32_t pData){
    if(!this->isOnline);
    long nErr = AdsSyncWriteReq(this->pAddr,ADSIGRP_SYM_VALBYHND, handlers[handle], sizeof(pData), &pData); 
    if (nErr) { cout << "Error: AdsSyncWriteReq: " << nErr << '\n'; }
}

void TwincatADSNode::WriteReq(string handle, bool* pData){
    if(!this->isOnline);
    long nErr = AdsSyncWriteReq(this->pAddr,ADSIGRP_SYM_VALBYHND, handlers[handle], sizeof(pData), &pData[0]); 
    if (nErr) { cout << "Error: AdsSyncWriteReq: " << nErr << '\n'; }
}

void TwincatADSNode::ReadReq(string handle, bool* pData){
    if(!this->isOnline);
    long nErr = AdsSyncReadReq(this->pAddr, ADSIGRP_SYM_VALBYHND, handlers[handle], sizeof(pData), &pData[0]);
    if (nErr) { cout << "Error: AdsSyncReadReq: " << nErr << '\n';}
}
