#include "..\include\GripperController.h"
#include <iostream>


GripperController::GripperController(){}

GripperController::GripperController(string portName, bool isOnline){
    this->isOnline = isOnline;
    this->node = ArduinoBLENode(portName);
    if(this->isOnline && !this->node.Connect()) { cout << "Failed to connect linear rail breaks. Exit programme.\n"; exit(-1); };
}
void GripperController::Open(){
    node.SendGripperSerial("(o,    )");
};
void GripperController::Close(){
    node.SendGripperSerial("(c,    )");
};
void GripperController::Rotate(int angle){
    string sendStr = "(r,";

    sendStr += ")"
    node.SendGripperSerial(sendStr);
};