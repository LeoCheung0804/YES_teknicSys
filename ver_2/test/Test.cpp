#include "..\include\Robot.h"
#include <iostream>
#include <string>
using namespace std;
int main(){
    // Test Robot Model and functions -- Done
    Robot robot("D:\\Galad_ws\\YES_teknicSys\\ver_2\\RobotConfig.json");
    cout << robot.IsValid() << endl;
    cout << robot.GetEEToGroundOffset() << endl;
    cout << robot.GetWorkingTrq() << endl;
    cout << robot.GetAbsTrqLmt() << endl;
    cout << robot.GetRailMotorNum() << endl;
    cout << robot.GetCableMotorNum() << endl;
    cout << robot.GetCableMotorScale() << endl;
    cout << robot.GetRailMotorScale() << endl;
    cout << robot.CheckLimits() << endl;
    robot.PrintEEPos();
    robot.EEPoseToCableLength(robot.endEffectorPos, robot.railOffset, robot.cableLength);
    robot.PrintCableLength();
    robot.PrintHomePos();
    cout << robot.CableMotorLengthToCmdAbsulote(10) << endl;
    cout << robot.CableMotorLengthToCmd(1, 10) << endl;
    cout << robot.RailMotorOffsetToCmd(1, 10) << endl;
   
}
