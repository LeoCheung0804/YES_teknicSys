#include "..\include\Robot.h"
#include "ArduinoBLENode.cpp"
#include "TwincatADSNode.cpp"
#include "TeknicNode.cpp"
#include <iostream>
#include <string>
using namespace std;

float workingTorque = -4.1; // cable working torque. 2.5 in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!

int main(){
    // Create Robot Model and read the config file
    Robot robot("D:\\Galad_ws\\YES_teknicSys\\ver_2\\RobotConfig.json");
    if(!robot.IsValid()) return -1; // quit if not creationg failed;
    workingTorque = robot.GetTargetTrq();

    // Init BLE Nodes
    ArduinoBLENode gripperNode(robot.GetGripperCommPort());
    ArduinoBLENode RailBreakNode(robot.GetRailBreakCommPort());

    // Init Rail Motor Nodes
    TwincatADSNode railNode();
    // Init Cable Motor Nodes
    TeknicNode cableNode(robot.GetCableMotorNum());
}
