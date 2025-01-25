#include "..\include\GripperController.h"
#include <iostream>
#include <assert.h>

float startAngle = -137;
float endAngle = startAngle + 180;
float startAngleTmp = startAngle;
float endAngleTmp = endAngle;
GripperController::GripperController(bool isOnline, bool useGripper){
    this->node = COMPortNode(isOnline);
    this->useGripper = useGripper;
}

void GripperController::Connect(string portName){
    if(!this->useGripper) return;
    if(!this->node.Connect(portName)) { 
        cout << "Error: Failed to connect gripper. \n"; 
        this->isConnected = false;
        return;
    }
    this->portName = portName;
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
    node.Send(this->sendStr);
    node.Send(this->sendStr);
    cout << "Gripper Opened. " << endl;
}

void GripperController::Close(){
    if(!this->useGripper) return;
    this->sendStr = "(c,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    node.Send(this->sendStr);
    node.Send(this->sendStr);
    node.Send(this->sendStr);
    cout << "Gripper Closed. " << endl;
}
void GripperController::Release(){
    if(!this->useGripper) return;
    this->sendStr = "(r,    )";
    // cout << "Sending Command: " << sendStr << " to gripper" << endl;
    node.Send(this->sendStr);
    node.Send(this->sendStr);
    node.Send(this->sendStr);
    cout << "Gripper Released. " << endl;
}

void GripperController::Rotate(int angle){
    if(!this->useGripper) return;
    // assert(angle >= -180 && angle <= 180);
    angle = startAngle + (endAngle - startAngle) * angle / 180;
    this->sendStr = "(d,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    node.Send(this->sendStr);
    node.Send(this->sendStr);
    node.Send(this->sendStr);
    cout << "Gripper Rotated to: " << angle << endl;
}
void GripperController::SetCalibrateStartAngle(int angle){
    if(!this->useGripper) return;
    // if(angle >= 200 || angle <= -200){
    //     cout << "Angle too large" << endl;
    //     return;
    // }
    this->sendStr = "(x,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    // node.Send(this->sendStr);
    startAngleTmp = angle;
    cout << "Gripper Rotated to: " << angle << endl;
}
void GripperController::SetCalibrateEndAngle(int angle){
    if(!this->useGripper) return;
    // if(angle >= 200 || angle <= -200){
        // cout << "Angle too large" << endl;
        // return;
    // }
    this->sendStr = "(y,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    // node.Send(this->sendStr);
    endAngleTmp = angle;
    cout << "Gripper Rotated to: " << angle << endl;
}

void GripperController::Calibrate(){
    if(!this->useGripper) return;
    // assert(angle >= -180 && angle <= 180);
    this->sendStr = "(w,";
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    // cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    // node.Send(this->sendStr);
    startAngle = startAngleTmp;
    endAngle = endAngleTmp;
    cout << "Gripper Calibrated " << endl;
}

void GripperController::Reset(){
    if(!this->useGripper) return;
    this->sendStr = "(h,  0 )";
    node.Send(this->sendStr);
    cout << "Sending Command: " << sendStr << " to gripper" << endl;
}

void GripperController::Reconnect(){
    if(!this->useGripper) return;
    this->node.Disconnect();
    cout << "Gripper Controller Offline." << endl;
    Sleep(1000);
    cout << "Reconnecting Gripper Controller..." << endl;
    if(!this->node.Connect(this->portName)) {
        cout << "Error: Failed to reconnect gripper. \n"; 
        this->isConnected = false;
        return;
    }
    cout << "Gripper Controller Online." << endl;
}