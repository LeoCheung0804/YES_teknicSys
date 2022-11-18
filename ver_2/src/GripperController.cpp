#include "..\include\GripperController.h"
#include <iostream>
#include <assert.h>

GripperController::GripperController(bool isOnline){
    this->isOnline = isOnline;
}

void GripperController::Connect(string portName){
    if(this->isOnline){
        this->node = ArduinoBLENode();
        if(!this->node.Connect(portName)) { 
            cout << "Error: Failed to connect gripper. \n"; 
            this->isConnected = false;
        }
    }
    cout << "Gripper Controller Online." << endl;
    this->isConnected = true;
}

void GripperController::Disconnect(){
    if(this->isOnline)
        this->node.Disconnect();
    cout << "Gripper Controller Offline." << endl;
    this->isConnected = false;
}

bool GripperController::IsConnected(){ return this->isConnected; }

void GripperController::Open(){
    this->sendStr = "(o,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    if(this->isOnline)
        node.Send(this->sendStr);
    cout << "Gripper Opened. " << endl;
}

void GripperController::Close(){
    this->sendStr = "(c,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    if(this->isOnline)
        node.Send(this->sendStr);
    cout << "Gripper Closed. " << endl;
}

void GripperController::Rotate(int angle){
    // assert(angle >= -180 && angle <= 180);
    this->sendStr = "(r,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    if(this->isOnline)
        node.Send(this->sendStr);
    cout << "Gripper Rotated to: " << angle << endl;
}

void GripperController::Calibrate(){
    this->sendStr = "(z,    )";
    cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    if(this->isOnline)
        node.Send(this->sendStr);
    else
        cout << "Offline mode, skip" << endl;
}