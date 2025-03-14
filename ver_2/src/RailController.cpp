#include "..\include\RailController.h"
#include <iostream>
#include <assert.h>

RailController::RailController(bool isOnline, bool useRail){ 
    this->motorNode = TwincatADSNode(isOnline);
    this->useRail = useRail; 
}

void RailController::Connect(int motorPortNumber, int railNumber){
    if(!this->useRail) return;
    this->railNumber = railNumber;
    this->bArry = new bool(railNumber);
    if(!this->motorNode.Connect(railNumber)){
        cout << "Failed to connect rail motors. Exit programme." << endl;
        this->isConnected = false;
    }
    cout << "Rail Controller Online." << endl;
    this->isConnected = true;
}

void RailController::Disconnect(){
    if(!this->useRail) return;
    this->motorNode.Disconnect();
    cout << "Rail Controller Offline." << endl;
    this->isConnected = false;
}

bool RailController::IsConnected(){ return this->isConnected; }

void RailController::SelectWorkingMotor(int index){
    index = (index + 2) % 4;
    if(!this->useRail) return;
    this->bArry[workingMotor] = false;
    this->bArry[index] = true;
    workingMotor = index;
}

void RailController::MoveMotorCmdAbsuloteById(int id, double cmd){
    
    id = (id + 2) % 4;
    if(!this->useRail) return;
    this->motorNode.WriteReq("MAIN.Axis_GoalPos", cmd);
    this->motorNode.WriteReq("MAIN.startMove[" + to_string(id + 1) + "]", true);
}

void RailController::MoveSelectedMotorCmdAbsulote(double cmd){
    if(!this->useRail) return;
    this->motorNode.WriteReq("MAIN.Axis_GoalPos", cmd);
    this->motorNode.WriteReq("MAIN.startMove[" + to_string(workingMotor + 1) + "]", true);

}

void RailController::CalibrationMotor(int index, double currentCmdPos){
    if(!this->useRail) return;
    bool *busyFlag = new bool(this->railNumber);
    bool *homeFlag = new bool(this->railNumber);
    
    for(int i = 0; i < this->railNumber; i++){
        busyFlag[i] = false;
        homeFlag[i] = false;
    }
    this->motorNode.WriteReq("MAIN.Axis_GoalPos", currentCmdPos); // write "MAIN.Axis_GoalPos"
    homeFlag[index] = true; // signal targeted rail motor for homing
    this->motorNode.WriteReq("MAIN.bHomeSwitch[" + to_string(index + 1) + "]", homeFlag); // write "MAIN.bHomeSwitch"
    homeFlag[index] = false; // return to false
    busyFlag[index] = false;

    while(busyFlag[index]){ // wait for motor busy flag on, ie. update current pos started
        cout << "Waiting for motor " << index << " to home" << endl;
        this->motorNode.ReadReq("MAIN.homeBusy", busyFlag);
    }
}

vector<int> RailController::GetMotorPosMeasured(){
    long nErr;
    bool *actPos = new bool(this->railNumber);
    vector<int> result;
    for(int i = 0; i < this->railNumber; i++){
        this->motorNode.ReadReq("MAIN.actPos[" + to_string(i + 1) + "]", actPos[i]);
        cout << "Rail " << i << " position: " << actPos[i] << endl;
        result.push_back((int)actPos[i]);
    }
    // vector<int> result(actPos, actPos + sizeof(actPos) / sizeof(actPos[0]));
    return result;
}

int RailController::GetMotorPosMeasuredById(int id){
    long nErr;
    double actPos = 0.0;
    this->motorNode.ReadReq("MAIN.actPos[" + to_string(id + 1) + "]", actPos);
    return actPos;
}
