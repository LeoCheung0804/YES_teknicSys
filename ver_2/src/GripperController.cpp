#include "..\include\GripperController.h"
#include <iostream>
#include <assert.h>

GripperController::GripperController(bool isOnline, bool useGripper){
    this->node = COMPortNode(isOnline);
    this->useGripper = useGripper;
}

void GripperController::Connect(string portName){
    if(!this->useGripper) return;
    if(!this->node.Connect(portName)) { 
        cout << "Error: Failed to connect gripper. \n"; 
        this->isConnected = false;
    }
    cout << "Gripper Controller Online." << endl;
    this->isConnected = true;
}

void GripperController::Disconnect(){
    if(!this->useGripper) return;
    this->node.Disconnect();
    cout << "Gripper Controller Offline." << endl;
    this->isConnected = false;
}

bool GripperController::IsConnected(){ return this->isConnected; }

void GripperController::Open(){
    if(!this->useGripper) return;
    this->sendStr = "(o,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    node.Send(this->sendStr);
    cout << "Gripper Opened. " << endl;
}

void GripperController::Close(){
    if(!this->useGripper) return;
    this->sendStr = "(c,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    node.Send(this->sendStr);
    cout << "Gripper Closed. " << endl;
}

void GripperController::Rotate(int angle){
    if(!this->useGripper) return;
    // assert(angle >= -180 && angle <= 180);
    this->sendStr = "(r,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    node.Send(this->sendStr);
    cout << "Gripper Rotated to: " << angle << endl;
}

void GripperController::Calibrate(){
    if(!this->useGripper) return;
    this->sendStr = "(z,    )";
    cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    node.Send(this->sendStr);
}