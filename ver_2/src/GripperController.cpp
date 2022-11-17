#include "..\include\GripperController.h"
#include <iostream>
#include <assert.h>


GripperController::GripperController(){}

GripperController::GripperController(string portName, bool isOnline){
    this->isOnline = isOnline;
    this->node = ArduinoBLENode(portName);
    if(this->isOnline && !this->node.Connect()) { cout << "Failed to connect linear rail breaks. Exit programme.\n"; exit(-1); };
}
void GripperController::Open(){
    this->sendStr = "(o,    )";
    cout << "Sending Command: " << sendStr << " to gripper" << endl;
    node.SendGripperSerial(this->sendStr);
};
void GripperController::Close(){
    this->sendStr = "(c,    )";
    cout << "Sending Command: " << sendStr << " to gripper" << endl;
    node.SendGripperSerial(this->sendStr);
};
void GripperController::Rotate(int angle){
    assert(angle >= -180 && angle <= 180);
    this->sendStr = "(r,";
    this->sendStr += to_string(angle);
    while(this->sendStr.length() < 7){
        this->sendStr += ' ';
    }
    this->sendStr += ")";
    cout << "Sending Command: " << this->sendStr << " to gripper" << endl;
    node.SendGripperSerial(this->sendStr);
};