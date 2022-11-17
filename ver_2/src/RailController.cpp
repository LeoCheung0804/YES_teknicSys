#include "..\include\RailController.h"
#include <iostream>
#include <assert.h>

RailController::RailController(){}

RailController::RailController(int motorPortNumber, string breakPortName, int railNumber, bool isOnline){
    this->isOnline = isOnline;
    this->railNumber = railNumber;
    this->bArry = new bool(railNumber);
    this->motorNode = TwincatADSNode(motorPortNumber);
    this->breakNode = ArduinoBLENode(breakPortName);
    if(this->isOnline && !this->motorNode.Connect()) { cout << "Failed to connect linear rail motors. Exit programme.\n"; exit(-1); };
    if(this->isOnline && !this->breakNode.Connect()) { cout << "Failed to connect linear rail breaks. Exit programme.\n"; exit(-1); };
}

void RailController::OpenBreak(int index){
    assert(index >= 0 && index <= railNumber);
    this->sendStr = "(0:0)   ";
    this->sendStr[3] = ('0' + index);
    cout << "Sending Command: " << this->sendStr << " to breaks" << endl;
    breakNode.SendGripperSerial(this->sendStr);
}
void RailController::CloseBreak(int index){
    assert(index >= 0 && index <= railNumber);
    this->sendStr = "(1:0)   ";
    this->sendStr[3] = ('0' + index);
    cout << "Sending Command: " << this->sendStr << " to breaks" << endl;
    breakNode.SendGripperSerial(this->sendStr);
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
    long nErr;
    nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handles["MAIN.Axis_GoalPos"], sizeof(cmd), &cmd); // write "MAIN.Axis_GoalPos"
    if (nErr) { cout << "Error: Rail[" << this->workingMotor << "] AdsSyncWriteReq: " << nErr << '\n'; }
    nErr = AdsSyncWriteReq(this->motorNode.pAddr,ADSIGRP_SYM_VALBYHND,this->motorNode.handles["MAIN.startMove"], sizeof(this->bArry), &this->bArry[0]); // write "MAIN.startMove"
    if (nErr) { cout << "Error: Rail[" << this->workingMotor << "] Start Absolute Move Command. AdsSyncWriteReq: " << nErr << '\n'; }
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