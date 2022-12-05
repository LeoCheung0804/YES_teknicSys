#include "..\include\BrakeController.h"
#include <iostream>
#include <assert.h>
#define RAIL_BRAKE_INDEX_START 0
#define CABLE_BRAKE_INDEX_START 4
#define RAIL_BRAKE_ALL 8
#define CABLE_BRAKE_ALL 9
BrakeController::BrakeController(bool isOnline){
    this->node = COMPortNode(isOnline);
    this->useCableBrake = false;
    this->useRailBrake = false;
}

void BrakeController::Connect(string portName){
    if(!this->useRailBrake) return;
    this->sendStr = "(0:0)   ";
    if(!this->node.Connect(portName)) { 
        cout << "Error: Failed to connect Brake. \n"; 
        this->isConnected = false;
    }
    Sleep(2000);
    cout << "Brake Controller Online." << endl;
    this->isConnected = true;
}

void BrakeController::Disconnect(){
    if(!this->useRailBrake) return;
    this->node.Disconnect();
    cout << "Brake Controller Offline." << endl;
    this->isConnected = false;
}

void BrakeController::UseCableBrake(int num){ 
    this->useCableBrake = true; 
    this->cableBrakeNum = num;
    this->cableBrakeFlag = new bool(num);
}

void BrakeController::UseRailBrake(int num){ 
    this->useRailBrake = true; 
    this->railBrakeNum = num;
    this->railBrakeFlag = new bool(num);
}

bool BrakeController::IsConnected(){ return this->isConnected; }

void BrakeController::OpenRailBrakeByIndex(int index){
    if(!this->useRailBrake) return;
    assert(index >= 0 && index <= this->railBrakeNum);
    this->sendStr[3] = '0';
    this->sendStr[1] = '0' + RAIL_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    this->node.Send(this->sendStr);
    this->railBrakeFlag[index] = false;
    Sleep(100);
    cout << "Rail Brake " << index << " Opened." << endl;
    
}

void BrakeController::CloseRailBrakeByIndex(int index){
    if(!this->useRailBrake) return;
    assert(index >= 0 && index <= this->railBrakeNum);
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + RAIL_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    this->node.Send(this->sendStr);
    this->railBrakeFlag[index] = true;
    Sleep(100);
    cout << "Rail Brake " << index << " Closed." << endl;
    
}

void BrakeController::OpenAllRailBrake(){
    if(!this->useRailBrake) return;
    for(int i = 0; i < this->railBrakeNum; i++){
        this->OpenRailBrakeByIndex(i);
    }
    Sleep(1000);
    cout << "ALL Rail Brake Opened." << endl;
    
}

void BrakeController::CloseAllRailBrake(){
    if(!this->useRailBrake) return;
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + RAIL_BRAKE_ALL;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    this->node.Send(this->sendStr);
        
    for(int i = 0; i < this->railBrakeNum; i++){
        this->railBrakeFlag[i] = true;
    }
    Sleep(100);
    cout << "ALL Rail Brake Closed." << endl;
    
}

void BrakeController::OpenCableBrakeByIndex(int index){
    if(!this->useCableBrake) return;
    assert(index >= 0 && index <= this->cableBrakeNum);
    this->sendStr[3] = '0';
    this->sendStr[1] = '0' + CABLE_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to cable brakes" << endl;
    this->node.Send(this->sendStr);
    Sleep(100);
    this->cableBrakeFlag[index] = false;
    cout << "Cable Brake " << index << " Opened." << endl;
}

void BrakeController::CloseCableBrakeByIndex(int index){
    if(!this->useCableBrake) return;
    assert(index >= 0 && index <= this->cableBrakeNum);
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + CABLE_BRAKE_INDEX_START + index;
    // cout << "Sending Command: " << this->sendStr << " to cable brakes" << endl;
    this->node.Send(this->sendStr);
    Sleep(100);
    this->cableBrakeFlag[index] = true;
    cout << "Cable Brake " << index << " Closed." << endl;
}

void BrakeController::OpenAllCableBrake(){
    if(!this->useCableBrake) return;
    for(int i = 0; i < this->cableBrakeNum; i++){
        this->OpenCableBrakeByIndex(i);
    }
    Sleep(1000);
    cout << "ALL Cable Brake Opened." << endl;
}

void BrakeController::CloseAllCableBrake(){
    if(!this->useCableBrake) return;
    this->sendStr[3] = '1';
    this->sendStr[1] = '0' + CABLE_BRAKE_ALL;
    // cout << "Sending Command: " << this->sendStr << " to Brakes" << endl;
    this->node.Send(this->sendStr);
    for(int i = 0; i < this->cableBrakeNum; i++){
        this->cableBrakeFlag[i] = true;
    }
    Sleep(100);
    cout << "ALL Cable Brake Closed." << endl;
}
