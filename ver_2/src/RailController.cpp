#include "..\include\RailController.h"
#include <iostream>
#include <assert.h>

RailController::RailController(bool isOnline){ this->isOnline = isOnline; }

void RailController::Connect(int motorPortNumber, int railNumber, string brakePortName, bool useMotor, bool useBraker){
    this->railNumber = railNumber;
    this->bArry = new bool(railNumber);
    this->brakeOnFlags = new bool(railNumber);
    this->motorNode = TwincatADSNode();
    this->brakeNode = ArduinoBLENode();
    this->isConnected = true;
    this->useMotor = useMotor;
    this->useBraker = useBraker;
    if(this->isOnline){
        if(useMotor){
            if(!this->motorNode.Connect(railNumber)){
                cout << "Failed to connect rail motors. Exit programme." << endl;
                this->isConnected = false;
            }
        }
        if(useBraker){
            if(!this->brakeNode.Connect(brakePortName)) { 
                cout << "Failed to connect rail motor brakes. Exit programme.\n"; 
                this->isConnected = false;
            };  
        }  
    }
    this->CloseAllBrake();
    cout << "Rail Controller Online." << endl;
}

void RailController::Disconnect(){
    this->CloseAllBrake();
    if(this->isOnline){
        if(this->useMotor)
            this->motorNode.Disconnect();
        if(this->useBraker)
            this->brakeNode.Disconnect();
    }
    cout << "Rail Controller Offline." << endl;
    this->isConnected = false;
}

bool RailController::IsConnected(){ return this->isConnected; }

void RailController::OpenBrake(int index){
    assert(index >= 0 && index <= railNumber);
    this->sendStr = "(0:0)   ";
    this->sendStr[3] = ('0' + index);
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    if(this->isOnline && this->useBraker)
        brakeNode.Send(this->sendStr);
    this->brakeOnFlags[index] = false;
    cout << "Rail Brake " << index << " Opened." << endl;
}

void RailController::CloseBrake(int index){
    assert(index >= 0 && index <= railNumber);
    this->sendStr = "(1:0)   ";
    this->sendStr[3] = ('0' + index);
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    if(this->isOnline && this->useBraker)
        brakeNode.Send(this->sendStr);
    this->brakeOnFlags[index] = true;
    cout << "Rail Brake " << index << " Closed." << endl;
}

void RailController::OpenAllBrake(){
    for(int i = 0; i < railNumber; i++){
        this->OpenBrake(i);
    }
};

void RailController::CloseAllBrake(){
    for(int i = 0; i < railNumber; i++){
        this->CloseBrake(i);
    }
};

void RailController::SelectWorkingMotor(int index){
    this->bArry[workingMotor] = false;
    this->bArry[index] = true;
    workingMotor = index;
}

void RailController::MoveSelectedMotorCmd(int32_t cmd, bool absolute){
    assert(!this->brakeOnFlags[workingMotor]);
    if(this->isOnline && this->useMotor){
        long nErr;
        nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handlers["MAIN.Axis_GoalPos"], sizeof(cmd), &cmd); // write "MAIN.Axis_GoalPos"
        if (nErr) { cout << "Error: Rail[" << this->workingMotor << "] AdsSyncWriteReq: " << nErr << '\n'; }
        nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handlers["MAIN.startMove"], sizeof(this->bArry), &this->bArry[0]); // write "MAIN.startMove"
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
    if(this->isOnline && this->useMotor){

        nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handlers["MAIN.Axis1_GoalPos"], sizeof(currentCmdPos), &currentCmdPos); // write "MAIN.Axis1_GoalPos"
        if (nErr) { cout << "Error: Rail[" << index << "] AdsSyncWriteReq: " << nErr << '\n'; }
        homeFlag[index] = true; // signal targeted rail motor for homing
        nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handlers["MAIN.bHomeSwitch"], sizeof(homeFlag), &homeFlag[0]); // write "MAIN.bHomeSwitch"
        if (nErr) { cout << "Error: Rail[" << index << "] Set postiton Command. AdsSyncWriteReq: " << nErr << '\n'; }
        homeFlag[index] = false; // return to false
        busyFlag[index] = false;
        while(!busyFlag[index]){ // wait for motor busy flag on, ie. update current pos started
            nErr = AdsSyncReadReq(this->motorNode.pAddr, ADSIGRP_SYM_VALBYHND, this->motorNode.handlers["MAIN.Axis_Home.Busy"], sizeof(busyFlag), &busyFlag[0]); // read "MAIN.Axis_Home.Busy"
            if (nErr) { cout << "Error: Rail[" << index << "] AdsSyncReadReq: " << nErr << '\n'; break; }
        }
        while(busyFlag[index]){ // wait for motor busy flag off, ie. completed updated position
            nErr = AdsSyncReadReq(this->motorNode.pAddr, ADSIGRP_SYM_VALBYHND, this->motorNode.handlers["MAIN.Axis_Home.Busy"], sizeof(busyFlag), &busyFlag[0]);
            if (nErr) { cout << "Error: Rail[" << index << "] AdsSyncReadReq: " << nErr << '\n'; break; }
        }
    }
}

vector<int> RailController::GetMotorPosMeasured(){
    if(this->isOnline && this->useMotor){
        long nErr;
        bool *actPos = new bool(this->railNumber);
        nErr = AdsSyncReadReq(this->motorNode.pAddr, ADSIGRP_SYM_VALBYHND, this->motorNode.handlers["MAIN.actPos"], sizeof(actPos), &actPos[0]); // read "MAIN.actPos"
        
        vector<int> result(actPos, actPos + sizeof(actPos) / sizeof(actPos[0]));
        if (nErr) { cout << "Error: Rail AdsSyncReadReq: " << nErr << '\n';  return result;}
        return result;
    }else{
        vector<int> result(6);
        return result;
    }
}