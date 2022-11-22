#pragma comment(lib, "User32.lib")
#include "..\include\Logger.h"
#include "..\include\Robot.h"
#include "..\include\TrajectoryGenerator.h"
#include <iostream>
#include <string>
#include <fstream>
#include <Windows.h>
using namespace std;

string userInput;
Robot robot;
Logger logger;

void ClearConsoleInputBuffer()
{
    // If you happen to have any trouble clearing already cleared buffer, uncomment the section below.
    /* keybd_event('S', 0, 0, 0);
    keybd_event('S', 0,KEYEVENTF_KEYUP, 0);
    keybd_event(VK_BACK, 0, 0, 0);
    keybd_event(VK_BACK, 0,KEYEVENTF_KEYUP, 0); */
    PINPUT_RECORD ClearingVar1 = new INPUT_RECORD[256];
    DWORD ClearingVar2;
    ReadConsoleInput(GetStdHandle(STD_INPUT_HANDLE),ClearingVar1,256,&ClearingVar2);
    delete[] ClearingVar1;
}

void PrintGripperControlMenu(){
    cout << "====================== Gripper Control Menu  ======================" << endl;
    cout << "\t1 - Close Gripper" << endl;
    cout << "\t2 - Open Gripper" << endl;
    cout << "\t3 - Rotate Gripper" << endl;
    cout << "\t4 - Calibrate Gripper" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void GripperControlMode(){    
    while(true){
        PrintGripperControlMenu();
        cin >> userInput;
        if(userInput == "q"){
            cout << "Bye" << endl;
            break;
        }else if(userInput == "1"){
            robot.gripper.Close();
            system("pause");
        }else if(userInput == "2"){
            robot.gripper.Open();
            system("pause");
        }else if(userInput == "3"){
            while(true){
                cout << "Please Enter Target Angle (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int angle = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        robot.gripper.Rotate(angle);
                    }else{
                        cout << "Please enter intager!!!";
                    }
                }
            }
        }else if(userInput == "4"){
            robot.gripper.Calibrate();
            system("pause");
        }
    }
}

void PrintCableMotorControlMenu(int selectedCable){
    cout << "====================== Cable Motor Control Mode  ======================" << endl;
    cout << "Beware that there is no soft emergency stop in this mode. " << endl;
    cout << "Current Selected Cable: " << selectedCable << endl;
    cout << "\t1 - Select Cable" << endl;
    cout << "\t2 - Move Selected Cable Relative (Motor Step)" << endl;
    cout << "\t3 - Move Selected Cable Absolute (Motor Step)" << endl;
    cout << "\t4 - Move Selected Cable Relative (Cable Length)" << endl;
    cout << "\t5 - Move Selected Cable Absolute (Cable Length)" << endl;
    cout << "\t6 - Move ALL Cable Relative (Motor Step)" << endl;
    cout << "\t7 - Move ALL Cable Absolute (Motor Step)" << endl;
    cout << "\t8 - Move ALL Cable Relative (Cable Length)" << endl;
    cout << "\t9 - Move ALL Cable Absolute (Cable Length)" << endl;
    cout << "\t10 - Open Selected Cable Brake" << endl;
    cout << "\t11 - Close Selected Cable Brake" << endl;
    cout << "\t12 - Open ALL Cable Brake" << endl;
    cout << "\t13 - Close ALL Cable Brake" << endl;
    cout << "\tq - Back To Pervioue Menu" << endl;
    cout << "Please Select Operation: " << endl;
}

void CableMotorControlMode(){
    int selectedCable = 0;
    while(true){
        PrintCableMotorControlMenu(selectedCable);
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Select Cable
            while(true){
                cout << "Please Enter Cable Index (0-7): ";
                cin >> userInput;
                char* p;
                int index = strtol(userInput.c_str(), &p, 10);
                if(!*p && index >=0 && index <=7) {
                    selectedCable = index;
                    cout << "Cable " << index << " Selected" << endl;
                    break;
                }else{
                    cout << "Please enter number 0 - 7!!!" << endl;
                }
            }
            system("pause");
        }else if(userInput == "2"){ // Move Selected Cable Relative By Motor Step
            while(true){
                cout << "Move Selected Cable Relative By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving Cable " << selectedCable << " By Step: " << step << endl;
                        robot.cable.MoveSingleMotorCmd(selectedCable, step, false);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "3"){ // Move Selected Cable Absolute By Motor Step
            while(true){
                cout << "Move Selected Cable Absolute By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving Cable " << selectedCable << " To Absolute Step: " << step << endl;
                        robot.cable.MoveSingleMotorCmd(selectedCable, step, true);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "4"){ // Move Selected Cable Relative By Cable Length
            while(true){
                cout << "Move Selected Cable Relative By Cable Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving Cable " << selectedCable << " By Length: " << length << endl;
                        robot.cable.MoveSingleMotorCmd(selectedCable, robot.CableMotorLengthToCmdAbsulote(length), false);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "5"){ // Move Selected Cable Absolute By Cable Length
            while(true){
                cout << "Move Selected Cable Absolute By Cable Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving Cable " << selectedCable << " To Absolute Length: " << length << endl;
                        robot.cable.MoveSingleMotorCmd(selectedCable, robot.CableMotorLengthToCmdAbsulote(length), true);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "6"){ // Move ALL Cable Relative By Motor Step
            while(true){
                cout << "Move ALL Cable Relative By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving ALL Cable By Step: " << step << endl;
                        robot.cable.MoveAllMotorCmd(step, false);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "7"){ // Move ALL Cable Absolute By Motor Step
            while(true){
                cout << "Move ALL Cable Absolute By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving ALL Cable To Absolute Step: " << step << endl;
                        robot.cable.MoveAllMotorCmd(step, true);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "8"){ // Move ALL Cable Relative By Cable Length
            while(true){
                cout << "Move ALL Cable Relative By Cable Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving ALL Cable By Length: " << length << endl;
                        robot.cable.MoveAllMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), false);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "9"){ // Move ALL Cable Absolute By Cable Length
            while(true){
                cout << "Move ALL Cable Absolute By Cable Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving ALL Cable To Absolute Length: " << length << endl;
                        robot.cable.MoveAllMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), true);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "10"){ // Open Selected Cable Brake
            robot.cable.OpenBrake(selectedCable);
            system("pause");
        }else if(userInput == "11"){ // Close Selected Cable Brake
            robot.cable.CloseBrake(selectedCable);
            system("pause");
        }else if(userInput == "12"){ // Open ALL Cable Brake
            for(int i = 0; i < robot.GetCableMotorBrakeNum(); i++)
                robot.cable.OpenBrake(i);
            system("pause");
        }else if(userInput == "13"){ // Close ALL Cable Brake
            for(int i = 0; i < robot.GetCableMotorBrakeNum(); i++)
                robot.cable.CloseBrake(i);
            system("pause");
        }
    }

}

void PrintRailMotorControlMenu(int selectedRailMotor){
    cout << "====================== Rail Motor Control Mode  ======================" << endl;
    cout << "Beware that there is no soft emergency stop in this mode. " << endl;
    cout << "Current Selected Rail: " << selectedRailMotor << endl;
    cout << "\t1 - Select Rail" << endl;
    cout << "\t2 - Move Selected Rail Relative (Motor Step)" << endl;
    cout << "\t3 - Move Selected Rail Absolute (Motor Step)" << endl;
    cout << "\t4 - Move Selected Rail Relative (Rail Length)" << endl;
    cout << "\t5 - Move Selected Rail Absolute (Rail Length)" << endl;
    cout << "\t6 - Move ALL Rail Relative (Motor Step)" << endl;
    cout << "\t7 - Move ALL Rail Absolute (Motor Step)" << endl;
    cout << "\t8 - Move ALL Rail Relative (Rail Length)" << endl;
    cout << "\t9 - Move ALL Rail Absolute (Rail Length)" << endl;
    cout << "\t10 - Open Selected Rail Brake" << endl;
    cout << "\t11 - Close Selected Rail Brake" << endl;
    cout << "\t12 - Open ALL Rail Brake" << endl;
    cout << "\t13 - Close ALL Rail Brake" << endl;
    cout << "\tq - Back To Pervioue Menu" << endl;
    cout << "Please Select Operation: " << endl;
}

void RailMotorControlMode(){
    int selectedRailMotor = 0;
    while(true){
        PrintRailMotorControlMenu(selectedRailMotor);
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Select Rail
            while(true){
                cout << "Please Enter Rail Index (0-4): ";
                cin >> userInput;
                char* p;
                int index = strtol(userInput.c_str(), &p, 10);
                if(!*p && index >=0 && index <=4) {
                    selectedRailMotor = index;
                    robot.rail.SelectWorkingMotor(index);
                    cout << "Rail " << index << " Selected" << endl;
                    break;
                }else{
                    cout << "Please enter number 0 - 4!!!" << endl;
                }
            }
            system("pause");
        }else if(userInput == "2"){ // Move Selected Rail Relative By Motor Step
            while(true){
                cout << "Move Selected Rail Relative By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving Rail " << selectedRailMotor << " By Step: " << step << endl;
                        robot.rail.MoveSelectedMotorCmd(step, false);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "3"){ // Move Selected Rail Absolute By Motor Step
            while(true){
                cout << "Move Selected Rail Absolute By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving Rail " << selectedRailMotor << " To Absolute Step: " << step << endl;
                        robot.rail.MoveSelectedMotorCmd(step, true);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "4"){ // Move Selected Rail Relative By Rail Length
            while(true){
                cout << "Move Selected Rail Relative By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving Rail " << selectedRailMotor << " By Length: " << length << endl;
                        robot.rail.MoveSelectedMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), false);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "5"){ // Move Selected Rail Absolute By Rail Length
            while(true){
                cout << "Move Selected Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving Rail " << selectedRailMotor << " To Absolute Length: " << length << endl;
                        robot.rail.MoveSelectedMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), true);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "6"){ // Move ALL Rail Relative By Motor Step
            // cout << "Currently not supported." << endl;
            while(true){
                cout << "Move ALL Rail Relative By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving ALL Rail By Step: " << step << endl;
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            robot.rail.SelectWorkingMotor(i);
                            robot.rail.MoveSelectedMotorCmd(step, false);
                        }
                        robot.rail.SelectWorkingMotor(selectedRailMotor);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "7"){ // Move ALL Rail Absolute By Motor Step
            // cout << "Currently not supported." << endl;
            while(true){
                cout << "Move ALL Rail Absolute By Motor Step" << endl;
                cout << "Please Enter Step (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    int step = strtol(userInput.c_str(), &p, 10);
                    if(!*p) {
                        cout << "Moving ALL Rail To Absolute Step: " << step << endl;
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            robot.rail.SelectWorkingMotor(i);
                            robot.rail.MoveSelectedMotorCmd(step, true);
                        }
                        robot.rail.SelectWorkingMotor(selectedRailMotor);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "8"){ // Move ALL Rail Relative By Rail Length
            // cout << "Currently not supported." << endl;
            while(true){
                cout << "Move ALL Rail Relative By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving ALL Rail By Length: " << length << endl;
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            robot.rail.SelectWorkingMotor(i);
                            robot.rail.MoveSelectedMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), false);
                        }
                        robot.rail.SelectWorkingMotor(selectedRailMotor);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "9"){ // Move ALL Rail Absolute By Rail Length
            // cout << "Currently not supported." << endl;
            while(true){
                cout << "Move ALL Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving ALL Rail To Absolute Length: " << length << endl;
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            robot.rail.SelectWorkingMotor(i);
                            robot.rail.MoveSelectedMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), true);
                        }
                        robot.rail.SelectWorkingMotor(selectedRailMotor);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "10"){ // Open Selected Rail Brake
            robot.rail.OpenBrake(selectedRailMotor);
            system("pause");
        }else if(userInput == "11"){ // Close Selected Rail Brake
            robot.rail.CloseBrake(selectedRailMotor);
            system("pause");
        }else if(userInput == "12"){ // Open ALL Rail Brake
            for(int i = 0; i < robot.GetRailMotorNum(); i++)
                robot.rail.OpenBrake(i);
            system("pause");
        }else if(userInput == "13"){ // Close ALL Rail Brake
            for(int i = 0; i < robot.GetRailMotorNum(); i++)
                robot.rail.CloseBrake(i);
            system("pause");
        }
    }

}

void PrintCalibrationMenu(){

    cout << "====================== Calibration Mode  ======================" << endl;
    cout << "\t1 - Tighten Cables" << endl;
    cout << "\t2 - Loose Cables" << endl;
    cout << "\t3 - Set All Motors to Home Position" << endl;
    cout << "\t4 - Reset EE Rotation to Zero" << endl;
    cout << "\t5 - Control Cable Motor " << endl;
    cout << "\t6 - Control Linear Rail Motor" << endl;
    cout << "\t7 - Control Gripper " << endl;
    cout << "\t8 - Update Robot Pos" << endl;
    cout << "\t9 - Update Robot Config" << endl;
    cout << "\t10 - Print Robot Status " << endl;
    cout << "\tq - Finish Calibration" << endl;
    cout << "Please Select Operation: " << endl;
}

void CalibrationMode(){ 
    while(true){
        PrintCalibrationMenu();
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Tighten Cables
            robot.cable.TightenAllCable(robot.GetWorkingTrq());
            system("pause");
        }else if(userInput == "2"){ // Loose Cables
            cout << "Loose Cables. Not implemented. Tell Galad he sucks." << endl;
            system("pause");
        }else if(userInput == "3"){ // Move All Cable Motors to Home Position
            robot.cable.HomeAllMotors();
            robot.rail.HomeAllMotors();
            system("pause");
        }else if(userInput == "4"){ // Reset End Effector Rotation to 0,0,0
            double targetPos[6] = {robot.endEffectorPos[0], robot.endEffectorPos[1], robot.endEffectorPos[2], 0, 0,0 };
            robot.cable.OpenAllBrake();
            robot.MoveToParaBlend(targetPos, 3500, false);
            robot.cable.CloseAllBrake();
            system("pause");
        }else if(userInput == "5"){ // Control Cable Individual
            robot.cable.OpenAllBrake();
            CableMotorControlMode();
            robot.cable.CloseAllBrake();
        }else if(userInput == "6"){ // Control Linear Rail Motor
            robot.cable.OpenAllBrake();
            RailMotorControlMode();
            robot.cable.CloseAllBrake();
        }else if(userInput == "7"){ // Control Gripper
            GripperControlMode();
        }else if(userInput == "8"){ // Update Robot Pos By File
            string robotPosPath;
            cout << "Please Enter Robot Position File Path (Default lastPos.txt): ";
            getchar();
            getline(cin, robotPosPath);
            robot.UpdatePosFromFile(robotPosPath  != "" ? robotPosPath : "lastPos.txt");
            logger.LogPos(robot.endEffectorPos, robot.railOffset);
            if(!robot.CheckLimits()){
                logger.LogWarning("New Position is beyond robot limit.");
                cout << "Warning: New Position is beyond robot limit. " << endl;
            }
            system("pause");
        }else if(userInput == "9"){ // Update Robot Config
            string robotConfigPath;
            cout << "Please Enter Robot Config File Path (Default RobotConfig.json): ";
            getchar();
            getline(cin, robotConfigPath);
            robot.UpdateModelFromFile(robotConfigPath  != "" ? robotConfigPath : "RobotConfig.json");
            while(!robot.IsValid()){
                cout << "Config file invalid, please use correct config file." << endl;
                string robotConfigFile;
                cout << "Please Enter Robot Configuration File Path (Default RobotConfig.json): ";
                getline(cin, robotConfigFile);
                robot.UpdateModelFromFile(robotConfigFile != "" ? robotConfigFile : "RobotConfig.json");
            }
            system("pause");
        }else if(userInput == "10"){ // print current status
            robot.PrintEEPos();
            robot.PrintRailOffset();
            robot.PrintCableLength();
            system("pause");
        }
    }
}

void PrintRobotPosControlMenu(){
    cout << "====================== Robot Pos Control Mode  ======================" << endl;
    cout << "Beware that there is no soft emergency stop in this mode. " << endl;
    cout << "\tw - Move the robot on +x direction." << endl;
    cout << "\ta - Move the robot on +y direction." << endl;
    cout << "\ts - Move the robot on -x direction." << endl;
    cout << "\td - Move the robot on -y direction." << endl;
    cout << "\tr - Move the robot on +z direction." << endl;
    cout << "\tf - Move the robot on -z direction." << endl;
    cout << "\tq - Exit Robot Pos Control Mode." << endl;
    cout << "=====================================================================" << endl;
}

void RobotPosControlMode(){
    PrintRobotPosControlMenu();
    robot.PrintEEPos();
    double goalPos[6] = {0};
    int sleepTime = 120;
    int moveTime = 100;
    while(true){
        if(GetKeyState('Q') & 0x8000)
        {
            Sleep(100);
            ClearConsoleInputBuffer();
            break;
        }else if(GetKeyState('A') & 0x8000) // +x
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[0] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('D') & 0x8000) // -x
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[0] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('W') & 0x8000) // +y
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[1] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('S') & 0x8000) // -y
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[1] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('R') & 0x8000) // +z
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[2] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('F') & 0x8000) // -z
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[2] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else{
            Sleep(sleepTime);
        }
    }
}

void PrintRailControlMenu(int selectedRail){
    
    cout << "====================== Rail Control Mode  ======================" << endl;
    cout << "Current Selected Rail: " << selectedRail << endl;
    cout << "\t1 - Select Rail" << endl;
    cout << "\t2 - Move Selected Rail Relative (Rail Length)" << endl;
    cout << "\t3 - Move Selected Rail Absolute (Rail Length)" << endl;
    cout << "\t4 - Move ALL Rail Relative (Rail Length)" << endl;
    cout << "\t5 - Move ALL Rail Absolute (Rail Length)" << endl;
    cout << "\tq - Back To Pervioue Menu" << endl;
    cout << "Please Select Operation: " << endl;
}

void RailControlMode(){
    
    int selectedRail = 0;
    while(true){
        PrintRailControlMenu(selectedRail);
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Select Rail
            while(true){
                cout << "Please Enter Rail Index (0-4): ";
                cin >> userInput;
                char* p;
                int index = strtol(userInput.c_str(), &p, 10);
                if(!*p && index >=0 && index <=4) {
                    selectedRail = index;
                    robot.rail.SelectWorkingMotor(index);
                    cout << "Rail " << index << " Selected" << endl;
                    break;
                }else{
                    cout << "Please enter number 0 - 4!!!" << endl;
                }
            }
            system("pause");
        }else if(userInput == "2"){ // Move Selected Rail Relative By Rail Length
            while(true){
                cout << "Move Selected Rail Relative By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving Rail " << selectedRail << " By Length: " << length << endl;
                        robot.MoveRail(selectedRail, length, false);
                        logger.LogInfo("Raise rail " + to_string(selectedRail) + " by " + to_string(length));
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "3"){ // Move Selected Rail Absolute By Rail Length
            while(true){
                cout << "Move Selected Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving Rail " << selectedRail << " To Absolute Length: " << length << endl;
                        robot.MoveRail(selectedRail, length, true);
                        logger.LogInfo("Raise rail " + to_string(selectedRail) + " to " + to_string(length));
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "4"){ // Move ALL Rail Relative By Rail Length
            // cout << "Currently not supported." << endl;
            while(true){
                cout << "Move ALL Rail Relative By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving ALL Rail By Length: " << length << endl;
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            robot.rail.SelectWorkingMotor(i);
                            robot.MoveRail(i, length, false);
                            logger.LogInfo("Raise rail " + to_string(i) + " by " + to_string(length));
                        }
                        robot.rail.SelectWorkingMotor(selectedRail);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "5"){ // Move ALL Rail Absolute By Rail Length
            // cout << "Currently not supported." << endl;
            while(true){
                cout << "Move ALL Rail Absolute By Rail Length" << endl;
                cout << "Please Enter Length in Meters (enter q to exit): ";
                cin >> userInput;
                if(userInput == "q"){
                    break;
                }else{
                    char* p;
                    double length = strtod(userInput.c_str(), &p);
                    if(!*p) {
                        cout << "Moving ALL Rail To Absolute Length: " << length << endl;
                        for(int i = 0; i < robot.GetCableMotorNum(); i++){
                            robot.rail.SelectWorkingMotor(i);
                            robot.MoveRail(i, length, true);
                            logger.LogInfo("Raise rail " + to_string(i) + " to " + to_string(length));
                        }
                        robot.rail.SelectWorkingMotor(selectedRail);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }
    }
}

void PrintRobotControlMenu(){
    cout << "====================== Robot Control Mode  ======================" << endl;
    cout << "\t1 - Robot Position Control" << endl;
    cout << "\t2 - Gripper Control" << endl;
    cout << "\t3 - Move Robot to Home Position" << endl;
    cout << "\t4 - Rail Control " << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void RobotControlMode(){
    while(true){
        PrintRobotControlMenu();
        cin >> userInput;

        if(userInput == "q"){ // back to last page
            break;
        }else if(userInput == "1"){ // robot position control mode
            robot.cable.OpenAllBrake();
            RobotPosControlMode();
            robot.cable.CloseAllBrake();
        }else if(userInput == "2"){ // gripper control mode
            GripperControlMode();
        }else if(userInput == "3"){ // move robot to home position
            robot.PrintHomePos();
            robot.cable.OpenAllBrake();
            robot.MoveToParaBlend(robot.homePos, false);
            robot.cable.CloseAllBrake();
            system("pause");
        }else if(userInput == "4"){ // Rail Control
            robot.cable.OpenAllBrake();
            robot.rail.OpenAllBrake();
            RailControlMode();
            robot.cable.CloseAllBrake();
            robot.rail.CloseAllBrake();
        }
    }
}

void PrintOperationMenu(){
    cout << "====================== Operation Mode  ======================" << endl;
    cout << "\t1 - Robot Control" << endl;
    cout << "\t2 - Read Brick Positions File" << endl;
    cout << "\t3 - Read Point-to-Point Path File" << endl;
    cout << "\t4 - Read Trajectory File" << endl;
    cout << "\t5 - Request current cable motor torque readings" << endl;
    cout << "\t6 - Save Current EE Pos to File" << endl;
    cout << "\t7 - Print Robot Status " << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void OperationMode(){
    while(true){
        PrintOperationMenu();
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Robot Pos Control
            RobotControlMode();
        }else if(userInput == "2"){ // Read brick position file
            string brickPosFileName;
            cout << "Please Enter Brick Pos File Path (Default bricks.csv): ";
            getchar();
            getline(cin, brickPosFileName);
            vector<vector<double>> brickPosList = ReadBrickPosFile(brickPosFileName != "" ? brickPosFileName : "bricks.csv", robot.rotationalAngleOffset, robot.rotationalDistanceOffset); // Read "bricks.csv"
            cout << "The file contains " << brickPosList.size() << " bricks." << endl;
            robot.PrintEEPos();
            robot.PrintBrickPickUpPos();
            cout << "Speed Limit: " << robot.GetVelLmt() << "m/s" << endl;
            double goalPos[6] = {0};
            int brickIndex = 0;
            robot.cable.OpenAllBrake();
            for(vector<double> brickPos : brickPosList){
                // Move to pre pick up position
                robot.gripper.Rotate(90);
                robot.gripper.Open();
                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                goalPos[5] += 0.0141; // calculated yaw for +0.21 height
                goalPos[2] += 0.21; // 0.21 safe height from ABB
                cout << "Brick No. " << brickIndex << endl;
                cout << "====== Moving to pre pick up position." << endl;
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                Sleep(100); // wait for brick

                // pickup brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                cout << "Brick No. " << brickIndex << endl;
                cout << "====== Picking up brick." << endl;
                if(!robot.MoveToParaBlend(goalPos, robot.safeT * 1.2, true)) break;
                Sleep(600); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
                robot.gripper.Close();
                Sleep(800); // wait for grippper to close

                // raise brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos+6, begin(goalPos));
                goalPos[2] += robot.GetEEToGroundOffset();
                cout << "====== Brick No. " << brickIndex << endl;
                cout << "====== Picking up brick." << endl;
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;

                // move to safe point
                robot.gripper.Rotate((int)(brickPos[4] + 92.2 - brickPos[3]/3.1415965*180)); // <-+27, constant frame to gripper offset; - yaw rotation in EE
                double safePt[6] = {8.24, 6.51, -2.7, 0, 0, -0.0237}; // a safe area near to the arm // 0.21 safe height from ABB
                copy(safePt, safePt+6, begin(goalPos)); // safe point
                cout << "====== Brick No. " << brickIndex << endl;
                cout << "====== Moving to safe position." << endl;
                if(!robot.MoveToParaBlend(goalPos, true)) break;

                double safeH = 0.12; // meter, safety height from building brick level
                // avoid the 5th pole
                /*
                double lowPole = -2.9; // lowest 5th pole z-value
                double safeZoneH = 0.4; // height of safe zone from bottom cable to top cable near EE
                if(brickPos[1]+2.149848*brickPos[0]>25.250178){ // check if target brick pos falls into the region where cables may hit 5th pole
                    cout << "Attention: Brick is in mountain region. Caution: may hit 5th pole\n"; 
                    // near road, check if 5th pole posistion need moving
                    if((lowPole-brickPos[2])*(lowPole-brickPos[2]+safeZoneH)<=0){
                        // Move 5th pole to just above the safe zone
                        // MoveBrkRail(brickPos[2]+safeZoneH);
                        goalPos[2] = lowPole; // safe level as 5th pole
                        cout << "Lowering to pole position??\n";
                        if(!robot.MoveToParaBlend(goalPos, false)) break;
                    }
                }
                else if(brickPos[1]-2.271415*brickPos[0]<-13.260362){ // check if target brick pos falls into the region where cables may hit 5th pole
                    cout << "Attention: Brick is in brick loading region. Caution: may hit 5th pole\n";
                    if((lowPole-brickPos[2])*(lowPole-brickPos[2]+safeZoneH)<=0){
                        // if lowPole < brickPos[2]+EeH { move ee to 5th pole height || move 5th above EE safeZone}
                        // else {move 5th pole to ee height}
                        // move to 5th pole height
                        goalPos[2] = lowPole; // safe level as 5th pole
                        cout << "Lowering to pole position??\n";
                        if(!robot.MoveToParaBlend(goalPos, false)) break;
                    }
                }
                // double safeH = 0.12; // meter, safety height from building brick level
                // goalPos[2] = brickPos[2] + robot.GetEEToGroundOffset() + safeH.; // brick level with safe height
                // if(robot.endEffectorPos[2] < brickPos[2] + robot.GetEEToGroundOffset() + safeH){ // if current position is below target brick height, then raise brick first // need this?
                //     if(!robot.MoveToParaBlend(goalPos, false)) break;
                // }
                */

                // Go to brick placing position
                goalPos[0] = brickPos[0];
                goalPos[1] = brickPos[1];
                goalPos[5] = brickPos[3]; // Include yaw angle
                cout << "====== Brick No. " << brickIndex << endl;
                cout << "====== Moving to brick position." << endl;
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                
                // Place brick
                goalPos[2] -= safeH;
                cout << "====== Brick No. " << brickIndex << endl;
                cout << "====== Placing brick." << endl;
                if(!robot.MoveToParaBlend(goalPos, robot.safeT, true)) break;
                robot.gripper.Open();
                Sleep(200); //Wait a while after placing brick

                // Rise and leave building area
                goalPos[2] += safeH;
                cout << "====== Brick No. " << brickIndex << endl;
                cout << "====== Moving to stand by position." << endl;
                if(!robot.MoveToParaBlend(goalPos, true)) break;

                // move to safe point
                copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y,z position
                cout << "====== Brick No. " << brickIndex << endl;
                cout << "====== Moving to safe position." << endl;
                if(robot.endEffectorPos[2] > safePt[2]){ // if current position is above safe point, then return to safe point within lowering z-height
                    goalPos[2] = robot.endEffectorPos[2];
                    if(!robot.MoveToParaBlend(goalPos, true)) break;
                    goalPos[2] = safePt[2];
                }
                if(!robot.MoveToParaBlend(goalPos, true)) break;
                cout << "----------Completed brick #" << brickIndex <<"----------" << endl;
                brickIndex++;
            }
            robot.cable.CloseAllBrake();
            system("pause");
        }else if(userInput == "3"){ // read point to point file, gen and run para blend traj
            string ptnFileName;
            cout << "Please Enter Point-To-Point Path File Path (Default points.csv): ";
            getchar();
            getline(cin, ptnFileName);
            vector<vector<double>> pointList = ReadPointFile(ptnFileName != "" ? ptnFileName : "points.csv"); // Read "bricks.csv"
            cout << "The file contains " << pointList.size() << " points." << endl;
            robot.cable.OpenAllBrake();
            for(vector<double> point : pointList){
                if(!robot.MoveToParaBlend(&point[0], true)) break;
            }
            robot.cable.CloseAllBrake();
            system("pause");
        }else if(userInput == "4"){ // read and run traj file
            string trajFileName;
            cout << "Please Enter Trajectory File Path (Default traj.csv): ";
            getchar();
            getline(cin, trajFileName);
            vector<vector<double>> trajList = ReadTrajFile(trajFileName != "" ? trajFileName : "traj.csv"); // Read "bricks.csv"
            cout << "The file contains " << trajList.size() << " points." << endl;
            robot.cable.OpenAllBrake();
            robot.RunCableTraj(trajList);
            robot.cable.CloseAllBrake();
            system("pause");
        }else if(userInput == "5"){ // requst current torque readings
            cout << "Current Measured Cable Motor Trq: " << endl;
            for(int i = 0; i < robot.GetCableMotorNum(); i++){
                cout << "\tCable " << i << ": " << robot.cable.GetMotorPosMeasured(i) << endl;
            }
            system("pause");
        }else if(userInput == "6"){ // save current ee pos to file
            string robotPosPath;
            cout << "Please Enter Robot Position File Path (Default lastPos.txt): ";
            getchar();
            getline(cin, robotPosPath);
            robot.SavePosToFile(robotPosPath  != "" ? robotPosPath : "lastPos.txt");
            system("pause");
        }else if(userInput == "7"){ // print current status
            robot.PrintEEPos();
            robot.PrintRailOffset();
            robot.PrintCableLength();
            system("pause");
        }
    }
}

void PrintMainMenu(){
    cout << "====================== Mode Selection  ======================" << endl;
    cout << "\t1 - Calibration Mode" << endl;
    cout << "\t2 - Operation Mode" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void MainMenu(){
    while(true){
        PrintMainMenu();
        cin >> userInput;
        if(userInput == "1"){
            CalibrationMode();
        }else if(userInput == "2"){
            OperationMode();
        }else if(userInput == "q"){
            cout << "Exiting..." << endl;
            break;
        }
    }
}

int main(){
    if(!logger.OpenFile("log\\robot.log")){
        cout << "Cannot open logger file. System will now exit." << endl;
        exit(-1);
    }
    
    logger.LogInfo("Reading robot config.");
    // read robot config file
    if(fstream("RobotConfig.json").good()){
        cout << "RobotConfig.json found. Config robot using RobotConfig.json " << endl;
        robot.UpdateModelFromFile("RobotConfig.json");
    }
    while(!robot.IsValid()){
        cout << "Config file invalid, please use correct config file." << endl;
        string robotConfigFile;
        cout << "Please Enter Robot Configuration File Path (Default RobotConfig.json): ";
        getline(cin, robotConfigFile);
        robot.UpdateModelFromFile(robotConfigFile != "" ? robotConfigFile : "RobotConfig.json");
    }
    system("pause");

    logger.LogInfo("Connecting to robot.");
    // connect to robot
    robot.Connect();

    if(robot.IsConnected()){
        cout << "All motors, all brakes and gripper connected success." << endl;
        logger.LogError("All motors, all brakes and gripper connected success.");
        
        // start main program
        MainMenu();
    }else{
        cout << "Robot Init Failed. Exiting..." << endl;
        logger.LogError("Rail connect failed.");
    }

    // disconnect from robot
    robot.Disconnect();
    logger.LogInfo("Robot Disconnected.");
    logger.CloseFile();
    cout << "Bye" << endl;

    return 0;
}