#include "..\include\Robot.h"
#include "..\include\TrajectoryGenerator.h"
#include <iostream>
#include <string>
using namespace std;

float workingTorque = -4.1; // cable working torque. 2.5 in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
bool isOnline = false;
string userInput;
Robot robot;

void PrintOffline(){
    cout << "!!!!!Warning!!!!!Offline Mode. If you see this, tell Galad he sucks. Or go to the source file and change bool isOnline to true then compile again." << endl;
}

void PrintMainMenu(){
    cout << "====================== Mode Selection  ======================" << endl;
    cout << "\t1 - Calibration Mode" << endl;
    cout << "\t2 - Operation Mode" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
}

void PrintOperationMenu(){
    cout << "====================== Operation Mode  ======================" << endl;
    cout << "\t1 - Calibration Mode" << endl;
    cout << "\t2 - Operation Mode" << endl;
    cout << "\tq - Exit" << endl;
    cout << "Please Select Mode: ";
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
            robot.cable.TightenAllCable(robot.GetTargetTrq());
            system("pause");
        }else if(userInput == "2"){ // Loose Cables
            cout << "Loose Cables. Not implemented. Tell Galad he sucks." << endl;
            system("pause");
        }else if(userInput == "3"){ // Move All Cable Motors to Home Position
            robot.cable.HomeAllCableMotors();
            system("pause");
        }else if(userInput == "4"){ // Reset End Effector Rotation to 0,0,0
            double targetPos[6] = {robot.endEffectorPos[0], robot.endEffectorPos[1], robot.endEffectorPos[2], 0, 0,0 };
            vector<vector<double>> cableTrajectory = GenParaBlendForCableMotor(robot.endEffectorPos, targetPos, 3500, false);
            robot.RunTraj(cableTrajectory);
            system("pause");
        }else if(userInput == "5"){ // Control Cable Individual
            
        }else if(userInput == "6"){ // Control Linear Rail Motor

        }else if(userInput == "7"){ // Control Gripper
            
        }else if(userInput == "8"){ // Update Robot Pos By File
            string robotPosPath;
            cout << "Please Enter Robot Position File Path (Default lastPos.txt): ";
            getchar();
            getline(cin, robotPosPath);
            robot.UpdatePosFromFile(robotPosPath  != "" ? robotPosPath : "RobotConfig.json");
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

void OperationMode(){
    while(true){
        PrintOperationMenu();
        cin >> userInput;
        if(userInput == "q"){ // Quit Calibration Mode
            break;
        }else if(userInput == "1"){
        }
    }
}

int main(){
    // Create Robot Model and read the config file
    robot = Robot("D:\\Galad_ws\\YES_teknicSys\\ver_2\\RobotConfig.json", isOnline);
    if(!robot.IsValid()) return -1; // quit if not creationg failed;
    workingTorque = robot.GetTargetTrq();


    robot.EEPoseToCableLength(robot.homePos); // save offset values according to home pose

    if(!isOnline)
        PrintOffline();
    cout << "All motors, all breaks and gripper connected success." << endl;

    while(true){
        PrintMainMenu();
        cin >> userInput;

        if(userInput == "1"){
            CalibrationMode();
        }else if(userInput == "2"){
            OperationMode();
        }else if(userInput == "q"){
            cout << "Bye" << endl;
            return 0;
        }
    }
    
    char cmd;
    

}