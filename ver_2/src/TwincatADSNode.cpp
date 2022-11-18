#include "..\include\TwincatADSNode.h"
#include <iostream>
TwincatADSNode::TwincatADSNode(){}

bool TwincatADSNode::Connect(int port){
    long nErr, nPort;
    unsigned long lHdlVar;
    bool TRUE_FLAG = true;

    cout << "Connecting to Twincat ADS Nodes and get handlers." << endl;
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
    map<string, unsigned long>::iterator iter;
    iter = handlers.begin();
    while(iter != handlers.end()){
        string handleName = iter->first;
        nErr = AdsSyncReadWriteReq(pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(lHdlVar), &lHdlVar, handleName.length(), "handleName.tc");
        if (nErr){ cout << "Error: AdsSyncReadWriteReq: " << nErr << ", in creating handler for " << iter->first << "\n"; return false; }
        handlers[handleName] = lHdlVar;
        iter++;
    }
    cout << "Completed creating handler list of " << handlers.size() << endl;
    return true;
}

void TwincatADSNode::Disconnect(){
    AdsPortClose();
}


