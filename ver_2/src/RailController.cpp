#include "..\include\RailController.h"
#include <iostream>


RailController::RailController(){}

RailController::RailController(int motorPortNumber, string breakPortName, int railNumber, bool isOnline){
    this->isOnline = isOnline;
    this->railNumber = railNumber;
    this->motorNode = TwincatADSNode(motorPortNumber);
    this->breakNode = ArduinoBLENode(breakPortName);
    if(this->isOnline && !this->motorNode.Connect()) { cout << "Failed to connect linear rail motors. Exit programme.\n"; exit(-1); };
    if(this->isOnline && !this->breakNode.Connect()) { cout << "Failed to connect linear rail breaks. Exit programme.\n"; exit(-1); };
}