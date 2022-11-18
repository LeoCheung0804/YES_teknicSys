#include "..\include\RailController.h"
#include <iostream>
#include <assert.h>

RailController::RailController(bool isOnline){ this->isOnline = isOnline; }

void RailController::Connect(int motorPortNumber, int railNumber, string brakePortName){
    this->railNumber = railNumber;
    this->bArry = new bool(railNumber);
    this->motorNode = TwincatADSNode();
    this->breakNode = ArduinoBLENode();
    if(this->isOnline){
        if(!this->motorNode.Connect(motorPortNumber)){ 
            cout << "Failed to connect linear rail motors. Exit programme.\n"; 
            this->isConnected = false;
        }
        if(!this->breakNode.Connect(brakePortName)){
            cout << "Failed to connect linear rail breaks. Exit programme.\n"; 
            this->isConnected = false;
        }
    }
    this->isConnected = true;
}

void RailController::Disconnect(){
    this->motorNode.Disconnect();
    this->breakNode.Disconnect();
    this->isConnected = false;
}

bool RailController::IsConnected(){ return this->isConnected; }

void RailController::OpenBreak(int index){
    assert(index >= 0 && index <= railNumber);
    this->sendStr = "(0:0)   ";
    this->sendStr[3] = ('0' + index);
    cout << "Sending Command: " << this->sendStr << " to breaks" << endl;
    if(this->isOnline)
        breakNode.Send(this->sendStr);
    else
        cout << "Offline mode, skip" << endl;
}

void RailController::CloseBreak(int index){
    assert(index >= 0 && index <= railNumber);
    this->sendStr = "(1:0)   ";
    this->sendStr[3] = ('0' + index);
    cout << "Sending Command: " << this->sendStr << " to breaks" << endl;
    if(this->isOnline)
        breakNode.Send(this->sendStr);
    else
        cout << "Offline mode, skip." << endl;
}

void RailController::HomeAllMotors(){
    for(int index = 0; index < this->railNumber; index++){
        this->CloseBreak(index);
    }
    for(int index = 0; index < this->railNumber; index++){
        this->OpenBreak(index);
        this->SelectWorkingMotor(index);
        this->MoveSelectedMotorCmd(0, true);
        this->CloseBreak(index);
    }
}

void RailController::SelectWorkingMotor(int index){
    this->bArry[workingMotor] = false;
    this->bArry[index] = true;
    workingMotor = index;
}

void RailController::MoveSelectedMotorCmd(int32_t cmd, bool absolute){
    if(this->isOnline){
        long nErr;
        nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handles["MAIN.Axis_GoalPos"], sizeof(cmd), &cmd); // write "MAIN.Axis_GoalPos"
        if (nErr) { cout << "Error: Rail[" << this->workingMotor << "] AdsSyncWriteReq: " << nErr << '\n'; }
        nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handles["MAIN.startMove"], sizeof(this->bArry), &this->bArry[0]); // write "MAIN.startMove"
        if (nErr) { cout << "Error: Rail[" << this->workingMotor << "] Start Absolute Move Command. AdsSyncWriteReq: " << nErr << '\n'; }
    }
}

void RailController::CalibrationMotor(int index, int32_t currentCmdPos){
    bool *busyFlag = new bool(this->railNumber);
    bool *homeFlag = new bool(this->railNumber);
    long nErr;
    
    for(int i = 0; i < this->railNumber; i++){
        busyFlag[i] = false;
        homeFlag[i] = false;
    }
    nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handles["MAIN.Axis1_GoalPos"], sizeof(currentCmdPos), &currentCmdPos); // write "MAIN.Axis1_GoalPos"
    if (nErr) { cout << "Error: Rail[" << index << "] AdsSyncWriteReq: " << nErr << '\n'; }
    homeFlag[index] = true; // signal targeted rail motor for homing
    nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handles["MAIN.bHomeSwitch"], sizeof(homeFlag), &homeFlag[0]); // write "MAIN.bHomeSwitch"
    if (nErr) { cout << "Error: Rail[" << index << "] Set postiton Command. AdsSyncWriteReq: " << nErr << '\n'; }
    homeFlag[index] = false; // return to false
    busyFlag[index] = false;
    while(!busyFlag[index]){ // wait for motor busy flag on, ie. update current pos started
        nErr = AdsSyncReadReq(this->motorNode.pAddr, ADSIGRP_SYM_VALBYHND, this->motorNode.handles["MAIN.Axis_Home.Busy"], sizeof(busyFlag), &busyFlag[0]); // read "MAIN.Axis_Home.Busy"
        if (nErr) { cout << "Error: Rail[" << index << "] AdsSyncReadReq: " << nErr << '\n'; break; }
    }
    while(busyFlag[index]){ // wait for motor busy flag off, ie. completed updated position
        nErr = AdsSyncReadReq(this->motorNode.pAddr, ADSIGRP_SYM_VALBYHND, this->motorNode.handles["MAIN.Axis_Home.Busy"], sizeof(busyFlag), &busyFlag[0]);
        if (nErr) { cout << "Error: Rail[" << index << "] AdsSyncReadReq: " << nErr << '\n'; break; }
    }
}

vector<int> RailController::GetMotorPosMeasured(){
    if(this->isOnline){
        long nErr;
        bool *actPos = new bool(this->railNumber);
        nErr = AdsSyncReadReq(this->motorNode.pAddr, ADSIGRP_SYM_VALBYHND, this->motorNode.handles["MAIN.actPos"], sizeof(actPos), &actPos[0]); // read "MAIN.actPos"
        
        vector<int> result(actPos, actPos + sizeof(actPos) / sizeof(actPos[0]));
        if (nErr) { cout << "Error: Rail AdsSyncReadReq: " << nErr << '\n';  return result;}
        return result;
    }else{
        vector<int> result(6);
        return result;
    }
}