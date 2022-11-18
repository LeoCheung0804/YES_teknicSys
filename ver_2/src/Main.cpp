#pragma comment(lib, "User32.lib")
#include "..\include\Robot.h"
#include "..\include\TrajectoryGenerator.h"
#include <iostream>
#include <string>
#include <Windows.h>
using namespace std;

string userInput;
Robot robot;

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

void PrintCableControlMenu(int selectedCable){
    cout << "====================== Cable Control Mode  ======================" << endl;
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
    cout << "\t10 - Open Selected Cable Break" << endl;
    cout << "\t11 - Close Selected Cable Break" << endl;
    cout << "\t12 - Open ALL Cable Break" << endl;
    cout << "\t13 - Close ALL Cable Break" << endl;
    cout << "\tq - Back To Pervioue Menu" << endl;
    cout << "Please Select Operation: " << endl;
}

void CableControlMode(){
    int selectedCable = 0;
    while(true){
        PrintCableControlMenu(selectedCable);
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
        }
        else if(userInput == "2"){ // Move Selected Cable Relative By Motor Step
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
        }else if(userInput == "10"){ // Open Selected Cable Break
            robot.cable.OpenBreak(selectedCable);
            system("pause");
        }else if(userInput == "11"){ // Close Selected Cable Break
            robot.cable.CloseBreak(selectedCable);
            system("pause");
        }else if(userInput == "12"){ // Open ALL Cable Break
            for(int i = 0; i < robot.GetCableMotorBreakNum(); i++)
                robot.cable.OpenBreak(i);
            system("pause");
        }else if(userInput == "13"){ // Close ALL Cable Break
            for(int i = 0; i < robot.GetCableMotorBreakNum(); i++)
                robot.cable.CloseBreak(i);
            system("pause");
        }
    }

}

void PrintRailControlMenu(int selectedRail){
    cout << "====================== Rail Control Mode  ======================" << endl;
    cout << "Current Selected Rail: " << selectedRail << endl;
    cout << "\t1 - Select Rail" << endl;
    cout << "\t2 - Move Selected Rail Relative (Motor Step)" << endl;
    cout << "\t3 - Move Selected Rail Absolute (Motor Step)" << endl;
    cout << "\t4 - Move Selected Rail Relative (Rail Length)" << endl;
    cout << "\t5 - Move Selected Rail Absolute (Rail Length)" << endl;
    cout << "\t6 - Move ALL Rail Relative (Motor Step)" << endl;
    cout << "\t7 - Move ALL Rail Absolute (Motor Step)" << endl;
    cout << "\t8 - Move ALL Rail Relative (Rail Length)" << endl;
    cout << "\t9 - Move ALL Rail Absolute (Rail Length)" << endl;
    cout << "\t10 - Open Selected Rail Break" << endl;
    cout << "\t11 - Close Selected Rail Break" << endl;
    cout << "\t12 - Open ALL Rail Break" << endl;
    cout << "\t13 - Close ALL Rail Break" << endl;
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
        }
        else if(userInput == "2"){ // Move Selected Rail Relative By Motor Step
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
                        cout << "Moving Rail " << selectedRail << " By Step: " << step << endl;
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
                        cout << "Moving Rail " << selectedRail << " To Absolute Step: " << step << endl;
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
                        cout << "Moving Rail " << selectedRail << " By Length: " << length << endl;
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
                        cout << "Moving Rail " << selectedRail << " To Absolute Length: " << length << endl;
                        robot.rail.MoveSelectedMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), true);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "6"){ // Move ALL Rail Relative By Motor Step
            cout << "Currently not supported." << endl;
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
                        // robot.rail.MoveSingleMotorCmd(step, false);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "7"){ // Move ALL Rail Absolute By Motor Step
            cout << "Currently not supported." << endl;
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
                        // robot.rail.MoveAllMotorCmd(step, true);
                    }else{
                        cout << "Please enter an intager number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "8"){ // Move ALL Rail Relative By Rail Length
            cout << "Currently not supported." << endl;
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
                        // robot.rail.MoveAllMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), false);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "9"){ // Move ALL Rail Absolute By Rail Length
            cout << "Currently not supported." << endl;
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
                        // robot.rail.MoveAllMotorCmd(robot.CableMotorLengthToCmdAbsulote(length), true);
                    }else{
                        cout << "Please enter a number!!!" << endl;
                    }
                }
            }
        }else if(userInput == "10"){ // Open Selected Rail Break
            robot.rail.OpenBreak(selectedRail);
            system("pause");
        }else if(userInput == "11"){ // Close Selected Rail Break
            robot.rail.CloseBreak(selectedRail);
            system("pause");
        }else if(userInput == "12"){ // Open ALL Rail Break
            for(int i = 0; i < robot.GetRailMotorNum(); i++)
                robot.rail.OpenBreak(i);
            system("pause");
        }else if(userInput == "13"){ // Close ALL Rail Break
            for(int i = 0; i < robot.GetRailMotorNum(); i++)
                robot.rail.CloseBreak(i);
            system("pause");
        }
    }

}

void PrintCalibrationMenu(){

    cout << "====================== Calibration Mode  ======================" << endl;
    // cout << "\tt - Tighten cables with Torque mode" << endl;
    // cout << "\ty - Loose the cables" << endl;
    // cout << "\th - Move to Home" << endl;
    // cout << "\t8 - Manually adjust cable lengths" << endl;
    // cout << "\tl - Linear rails motions" << endl;
    // cout << "\tu - Update current position from external file" << endl;
    // cout << "\tr - Reset Rotation to zero" << endl;
    // cout << "\tg - Gripper functions" << endl;
    // cout << "\tj - Read Json file and update model params" << endl;
    // cout << "\ti - Info: show menu" << endl;
    // cout << "\tn - Move on to Next step" << endl;
    cout << "\t1 - Tighten Cables" << endl;
    cout << "\t2 - Loose Cables" << endl;
    cout << "\t3 - Set All Motors to Home Position" << endl;
    cout << "\t4 - Reset EE Rotation to Zero" << endl;
    cout << "\t5 - Control Cable Motor " << endl;
    cout << "\t6 - Control Linear Rail Motor" << endl;
    cout << "\t7 - Control Gripper " << endl;
    cout << "\t8 - Update Robot Pos" << endl;
    cout << "\t9 - Update Robot Config" << endl;
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
            robot.MoveToParaBlend(targetPos, 3500, false);
            system("pause");
        }else if(userInput == "5"){ // Control Cable Individual
            CableControlMode();
        }else if(userInput == "6"){ // Control Linear Rail Motor
            RailControlMode();
        }else if(userInput == "7"){ // Control Gripper
            GripperControlMode();
        }else if(userInput == "8"){ // Update Robot Pos By File
            string robotPosPath;
            cout << "Please Enter Robot Position File Path (Default lastPos.txt): ";
            getchar();
            getline(cin, robotPosPath);
            robot.UpdatePosFromFile(robotPosPath  != "" ? robotPosPath : "lastPos.txt");
            system("pause");
        }else if(userInput == "9"){ // Update Robot Config
            string robotConfigPath;
            cout << "Please Enter Robot Config File Path (Default RobotConfig.json): ";
            getchar();
            getline(cin, robotConfigPath);
            robot.UpdateModelFromFile(robotConfigPath  != "" ? robotConfigPath : "RobotConfig.json");
            system("pause");
        }
    }
}

void PrintRobotPosControlMenu(){
    cout << "====================== Robot Pos Control Mode  ======================" << endl;
    cout << "\tw - Move the robot on +x direction." << endl;
    cout << "\ta - Move the robot on +y direction." << endl;
    cout << "\ts - Move the robot on -x direction." << endl;
    cout << "\td - Move the robot on -y direction." << endl;
    cout << "\tr - Move the robot on +z direction." << endl;
    cout << "\tf - Move the robot on -z direction." << endl;
    cout << "\tq - Exit" << endl;
}

void RobotPosControlMode(){
    PrintRobotPosControlMenu();
    cout << "=====================================================================" << endl;
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
            robot.MoveToLinear(goalPos, moveTime, false);
            
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('D') & 0x8000) // -x
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[0] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('W') & 0x8000) // +y
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[1] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('S') & 0x8000) // -y
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[1] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('R') & 0x8000) // +z
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[2] += 0.01;
            robot.MoveToLinear(goalPos, moveTime, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else if(GetKeyState('F') & 0x8000) // -z
        {
            copy(robot.endEffectorPos, robot.endEffectorPos + 6, goalPos);
            goalPos[2] -= 0.01;
            robot.MoveToLinear(goalPos, moveTime, false);
            printf("\x1b[7A"); // eepos 7 + press key 1
            robot.PrintEEPos();
            Sleep(sleepTime);
        }else{
            Sleep(sleepTime);
        }
    }
}

void PrintRobotControlMenu(){
    cout << "====================== Robot Control Mode  ======================" << endl;
    cout << "\t1 - Robot Position Control" << endl;
    cout << "\t2 - Gripper Control" << endl;
    cout << "\t3 - Move Robot to Home Position" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void RobotControlMode(){
    while(true){
        PrintRobotControlMenu();
        cin >> userInput;

        if(userInput == "1"){ // robot position control mode
            RobotPosControlMode();
        }else if(userInput == "2"){ // gripper control mode
            GripperControlMode();
        }else if(userInput == "3"){ // move robot to home position
            double targetPos[6] = { 0, 0, 0, 0, 0, 0 };
            robot.MoveToParaBlend(targetPos, false);
            system("pause");
        }else if(userInput == "q"){
            cout << "Bye" << endl;
            break;
        }
    }
}

void PrintOperationMenu(){
    cout << "====================== Operation Mode  ======================" << endl;
    cout << "\t1 - Read Brick Positions File" << endl;
    cout << "\t2 - Robot Control" << endl;
    cout << "\t3 - Read Point-to-Point Path File" << endl;
    cout << "\t4 - Read Trajectory File" << endl;
    cout << "\t5 - Request current cable motor torque readings" << endl;
    cout << "\t6 - Save Current EE Pos to File" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void OperationMode(){
    while(true){
        PrintOperationMenu();
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){ // Read brick position file
            string brickPosFileName;
            cout << "Please Enter Robot Config File Path (Default bricks.csv): ";
            getchar();
            getline(cin, brickPosFileName);
            vector<vector<double>> brickPosList = ReadBrickPosFile(brickPosFileName != "" ? brickPosFileName : "bricks.csv", robot.rotationalAngleOffset, robot.rotationalDistanceOffset); // Read "bricks.csv"
            cout << "The file contains " << brickPosList.size() << " bricks." << endl;
            robot.PrintEEPos();
            robot.PrintBrickPickUpPos();
            cout << "Speed Limit: " << robot.GetVelLmt() << "m/s" << endl;
            double goalPos[6] = {0};
            int brickIndex = 0;
            for(vector<double> brickPos : brickPosList){
                // Move to pre pick up position
                robot.gripper.Rotate(90);
                robot.gripper.Open();
                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                goalPos[5] += 0.0141; // calculated yaw for +0.21 height
                goalPos[2] += 0.21; // 0.21 safe height from ABB
                robot.MoveToParaBlend(goalPos, false);
                Sleep(100); // wait for brick

                // pickup brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos + 6, goalPos);
                robot.MoveToParaBlend(goalPos, robot.safeT * 1.2, false);
                Sleep(600); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
                robot.gripper.Close();
                Sleep(800); // wait for grippper to close

                // raise brick
                copy(robot.brickPickUpPos, robot.brickPickUpPos+6, begin(goalPos));
                goalPos[2] += robot.GetEEToGroundOffset();
                robot.MoveToParaBlend(goalPos, robot.safeT, false);

                // move to save point
                robot.gripper.Rotate((int)(brickPos[4] + 92.2 - brickPos[3]/3.1415965*180)); // <-+27, constant frame to gripper offset; - yaw rotation in EE
                double safePt[6] = {8.24, 6.51, -2.7, 0, 0, -0.0237}; // a safe area near to the arm // 0.21 safe height from ABB
                copy(safePt, safePt+6, begin(goalPos)); // safe point
                robot.MoveToParaBlend(goalPos, false);

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
                        robot.MoveToParaBlend(goalPos, false);
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
                        robot.MoveToParaBlend(goalPos, false);
                    }
                }
                // double safeH = 0.12; // meter, safety height from building brick level
                // goalPos[2] = brickPos[2] + robot.GetEEToGroundOffset() + safeH.; // brick level with safe height
                // if(robot.endEffectorPos[2] < brickPos[2] + robot.GetEEToGroundOffset() + safeH){ // if current position is below target brick height, then raise brick first // need this?
                //     robot.MoveToParaBlend(goalPos, false);
                // }
                */

                // Go to brick placing position
                cout << "Going to brick position" << endl;
                goalPos[0] = brickPos[0];
                goalPos[1] = brickPos[1];
                goalPos[5] = brickPos[3]; // Include yaw angle
                robot.MoveToParaBlend(goalPos, false);
                
                // Place brick
                cout << "Placing brick" << endl; 
                goalPos[2] -= safeH;
                robot.MoveToParaBlend(goalPos, robot.safeT, false);
                robot.gripper.Open();
                Sleep(200); //Wait a while after placing brick

                // Rise and leave building area
                cout << "Going to stand by position\n"; 
                goalPos[2] += safeH;
                robot.MoveToParaBlend(goalPos, false);

                // move to safe point
                copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y,z position
                if(robot.endEffectorPos[2] > safePt[2]){ // if current position is above safe point, then return to safe point within lowering z-height
                    goalPos[2] = robot.endEffectorPos[2];
                    robot.MoveToParaBlend(goalPos, false);
                    goalPos[2] = safePt[2];
                }
                robot.MoveToParaBlend(goalPos, false);

                // log
                robot.PrintEEPos();
                robot.PrintRailOffset();
                cout << "----------Completed brick #" << brickIndex <<"----------" << endl;
                brickIndex++;
            }

            system("pause");
            break;
        }else if(userInput == "2"){ // Robot Pos Control
            RobotControlMode();
        }else if(userInput == "3"){ // read point to point file, gen and run para blend traj
            string ptnFileName;
            cout << "Please Enter Point-To-Point Path File Path (Default points.csv): ";
            getchar();
            getline(cin, ptnFileName);
            vector<vector<double>> pointList = ReadPointFile(ptnFileName != "" ? ptnFileName : "points.csv"); // Read "bricks.csv"
            cout << "The file contains " << pointList.size() << " points." << endl;
            for(vector<double> point : pointList){
                robot.MoveToParaBlend(&point[0], true);
            }
            system("pause");
        }else if(userInput == "4"){ // read and run traj file
            string trajFileName;
            cout << "Please Enter Trajectory File Path (Default traj.csv): ";
            getchar();
            getline(cin, trajFileName);
            vector<vector<double>> trajList = ReadTrajFile(trajFileName != "" ? trajFileName : "traj.csv"); // Read "bricks.csv"
            cout << "The file contains " << trajList.size() << " points." << endl;
            robot.RunTraj(trajList);
            system("pause");
        }else if(userInput == "5"){ // requst current torque readings
            cout << "Current Measured Cable Motor Trq: " << endl;
            for(int i = 0; i < robot.GetCableMotorNum(); i++){
                cout << "\tCable " << i << ": " << robot.cable.GetMotorPosMeasured(i) << endl;
            }
        }else if(userInput == "6"){ // save current ee pos to file
            string robotPosPath;
            cout << "Please Enter Robot Position File Path (Default lastPos.txt): ";
            getchar();
            getline(cin, robotPosPath);
            robot.SavePosToFile(robotPosPath  != "" ? robotPosPath : "RobotConfig.json");
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
            cout << "Bye" << endl;
            break;
        }
    }
}

int main(){

    // read robot config file
    string robotConfigFile;
    cout << "Please Enter Robot Configuration File Path (Default RobotConfig.json): ";
    getline(cin, robotConfigFile);
    robot.UpdateModelFromFile(robotConfigFile != "" ? robotConfigFile : "RobotConfig.json");
    system("pause");

    // connect to robot
    robot.Connect();

    // start main program
    MainMenu();

    // disconnect from robot
    robot.Disconnect();

    return 0;
}