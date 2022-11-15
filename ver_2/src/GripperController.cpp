#include "..\include\GripperController.h"
#include <iostream>


GripperController::GripperController(){}

GripperController::GripperController(string portName, bool isOnline){
    this->isOnline = isOnline;
    this->node = ArduinoBLENode(portName);
    if(this->isOnline && !this->node.Connect()) { cout << "Failed to connect linear rail breaks. Exit programme.\n"; exit(-1); };
}