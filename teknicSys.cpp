#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <conio.h>
#include <Windows.h>
#include "CDPR.h"
#include "ABBgetVar.h"
#include "Dependencies\sFoundation20\inc\pubSysCls.h"
// #include "Dependencies\DynamixelSDK-3.7.31\include\dynamixel_sdk\dynamixel_sdk.h"

using namespace std;
using namespace sFnd;

bool SetSerialParams(HANDLE hComm);
void SendGripperSerial(HANDLE hComm, unsigned char* Ard_char);
int CheckMotorNetwork();
int RunParaBlend(CDPR &r, double point[7], bool showAttention = false);
void RunBricksTraj(HANDLE hComm, CDPR &r, unsigned char* Ard_char, int listOffset, bool showAttention = false, bool waitBtn = false);
void ReverseBricksTraj(CDPR &r, int listOffset, bool showAttention = false);
void RunTrajPoints(CDPR &r);
void SendMotorGrp(CDPR &r, bool IsTorque = false, bool IsLinearRail = false);
void TrjHome(CDPR &r);
bool ReadBricksFile();
void MN_DECL AttentionDetected(const mnAttnReqReg &detected); // this is attention from teknic motors

vector<string> comHubPorts;
vector<INode*> nodeList; // create a list for each node
vector<vector<double>> brickPos;
unsigned int portCount;
char attnStringBuf[512]; // Create a buffer to hold the attentionReg information    
const double RAIL_UP = 1.25, RAIL_DOWN = 0; // Linear rail upper and lower bound
double step = 0.01; // in meters, for manual control
float targetTorque = -4.1; //2.5 in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
const int MILLIS_TO_NEXT_FRAME = 35, UserInput_Sec_Timeout = 15, SleepTime = 21; // note the basic calculation time is abt 16ms; sleep-time in 24 hr
double railOffset = 0.0; // linear rails offset
char limitType = 'C'; // A for home, B for limits, C for default
char quitType = 'r'; // q for emergency quit, f for finish traj, e for error msg received, r for resume/default
int loopCount = 0, abbCount = 0; // log info for loop numbers and error occurs daily

// bool groupSyncRead = true;

int main()
{   
    CDPR robot; // Read model.json and create object
    if(!robot.IsGood()){ return -1; } // quit programme if object creation
    robot.PrintHome();
    targetTorque = robot.TargetTorque();
    
    // Local varia)bles for COM port communication
    HANDLE hComm; // Handle to the Serial port, https://github.com/xanthium-enterprises/Serial-Programming-Win32API-C
    char ComPortName[] = "\\\\.\\COM9"; // Name of the arduino Serial port(May Change) to be opened,
    unsigned char Ard_char[8] = {'(','o',',',' ',' ',' ',' ',')'};

    //// Open serial port for Arduino with bluetooth
    hComm = CreateFile(ComPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hComm == INVALID_HANDLE_VALUE){ cout << "Error: " << ComPortName << " cannot be opened.\n"; }
    else { cout << ComPortName << " opened.\n"; }
    if (!SetSerialParams(hComm)) { return -1; }
    
    //// Initiallize cable robot motor network
    SysManager* myMgr = SysManager::Instance();
    // Start the programme, scan motors in network
    try{
        if (CheckMotorNetwork() < 0){
            cout << "Motor network not available. Exit programme." << endl;
            return -1;
        }
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        return -1;
    }
    // IPort &myPort = myMgr->Ports(2);
    robot.PoseToLength(robot.home, robot.offset); // save offset values according to home pose

    cout << "Motor network available. Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\nh - Move to Home\n8 - Manually adjust cable lengths\nl - Linear rails motions\nu - Update current position from external file\nr - Reset Rotation to zero\ng - Gripper functions\ni - Info: show menu\nn - Move on to Next step" << endl;
    char cmd;
    do {
        try{
            do {
                bool allDone = false, stabilized = false;
                cin >> cmd;
                switch (cmd){
                    case 'i':   // Show menu
                        cout << "Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\nh - Move to Home\n8 - Manually adjust cable lengths\nl - Linear rails motions\nu - Update current position from external file\nr - Reset Rotation to zero\ng - Gripper functions\ni - Info: show menu\nn - Move on to Next step\n";
                        break;
                    case 'y':   // Loosen cables using positive torque
                        targetTorque = 1;
                    case 't':   // Tighten cables according to torque **Only for 8 motors
                        cout << "Current target torque = " << targetTorque << endl;
                        for(INode* n : nodeList){ n->Motion.AccLimit = 200; }
                        while(!stabilized) {
                            SendMotorGrp(robot, true);
                            Sleep(50);
                            for (int n = 0; n < robot.NodeNum(); n++){
                                if(nodeList[n]->Motion.TrqCommanded.Value() > targetTorque) { break; }
                                stabilized = true;
                            }
                        } 
                        for(INode* n : nodeList){
                            n->Motion.TrqCommanded.Refresh();
                            cout << n->Motion.TrqCommanded.Value() << "\t";
                            n->Motion.MoveVelStart(0);
                            n->Motion.AccLimit = 40000;
                        }
                        cout << "\nTorque mode tightening completed" << endl;
                        targetTorque = robot.TargetTorque();
                        break;
                    // case 's':   // Set zero
                    //     for (int n = 0; n < NodeNum; n++){
                    //         nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value()); // Zeroing the number space around the current Measured Position
                    //     }
                    //     copy(begin(home), end(home), begin(in1)); // copy home array into input array
                    //     cout << "Setting zero completed" << endl;
                    //     cout <<  "Home coordinates: " << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;
                    //
                    //     cout << "Do you want to set current rail position as zero? (k - OK)\n";;
                    //     cin >> cmd;
                    //     if(cmd == 'k'){
                    //         for (int n = NodeNum; n < NodeNum+4; n++){
                    //             nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value());
                    //         }
                    //         cout << "Linear rails are set to zero.\n";
                    //     }
                    //     else{ cout << "Current rail position may not be at zero\n"; }
                    //     break;
                    case 'h':   // Homing for all motors!! Including linear rail
                        allDone = false;
                        for (int n = 0; n<nodeList.size(); n++) { 
                            nodeList[n]->Motion.MoveWentDone();
                            nodeList[n]->Motion.MovePosnStart(0, true); // absolute position
                        }
                        while(!allDone) {
                            for (INode* n : nodeList) {
                                if(!n->Motion.MoveIsDone()) { break; }
                                allDone = true;
                            }
                        }
                        copy(begin(robot.home), end(robot.home), begin(robot.in)); // copy home array into input array
                        cout << "Homing completed" << endl;
                        break;
                    case '8':   // Manual cable adjustment
                        cout << "0 to 7 - motor id to adjust cable length\n8 - move 4 linear rails together\na or d - increase or decrease cable length\nb - Back to previous menu\n";
                        while(cmd != 'b'){
                            cin >> cmd;
                            if('/' < cmd && cmd < robot.NodeNum() + 49){
                                int id = cmd - 48;
                                int sCount = robot.ToMotorCmd(-1, step) / 5 ;// = ToMotorCmd(robot, -1, step) / 5;
                                cout << "Motor "<< cmd <<" selected.\n";
                                do{
                                    cmd = getch();
                                    switch(cmd){
                                        case 'a':
                                            if(id == robot.NodeNum()){ for(int n = robot.NodeNum(); n<robot.NodeNum()+4; n++){ nodeList[n]->Motion.MovePosnStart(sCount); }}
                                            else { nodeList[id]->Motion.MovePosnStart(sCount); }
                                            break;
                                        case 'd':
                                            if(id == robot.NodeNum()){ for(int n = robot.NodeNum(); n<robot.NodeNum()+4; n++){ nodeList[n]->Motion.MovePosnStart(-sCount); }}
                                            else { nodeList[id]->Motion.MovePosnStart(-sCount); }
                                            break;
                                        case 'i':
                                            if(id == robot.NodeNum()){
                                                for(int n = robot.NodeNum(); n<robot.NodeNum()+4; n++){
                                                    nodeList[n]->Motion.PosnMeasured.Refresh();
                                                    cout << (double) nodeList[n]->Motion.PosnMeasured << endl;
                                                }
                                                cout << endl;
                                            }
                                            else{
                                                nodeList[id]->Motion.PosnMeasured.Refresh();
                                                cout << (double) nodeList[id]->Motion.PosnMeasured << endl;
                                            }
                                            break;
                                        case 'h':
                                            if(id == robot.NodeNum()){ cout << "Homing for linear rails are not implemented here.\n"; break; }
                                            nodeList[id]->Motion.VelLimit = 300;
                                            nodeList[id]->Motion.MoveWentDone();
                                            nodeList[id]->Motion.MovePosnStart(0, true);
                                            while(!nodeList[id]->Motion.MoveIsDone()){}
                                            cout << "Individual homing completed.\n";
                                            nodeList[id]->Motion.VelLimit = 3000;
                                            break;
                                    }                        
                                    Sleep(100); // do we need this?
                                }while(cmd =='a'|| cmd =='d' || cmd =='h' || cmd =='i');
                                cout << "Motor "<< id <<" deselected.\n";
                            }
                        }
                        cout << "Manual adjustment terminated" << endl;
                        break;
                    case 'r':
                        cout << "Attenion: robot will rotate to 0,0,0...\n";
                        {
                            double point[7] = {0,0,0,0,0,0,3500}; // Default 3500 ms duaration to rotate to 0
                            copy(robot.in, robot.in+3, begin(point)); // copy x,y,z position
                            cout << "Goal coordinates: " << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << ", " << point[4] << ", " << point[5] << ", " << point[6] << endl;
                            if(RunParaBlend(robot, point) < 0) { cout << "Trajectory aborted.\n"; break; }
                            cout << "Rotation reset completed.\n";      
                        }
                        break;
                    case 'g':
                        cout << "In gripper testing mode:\ni - close gripper\no = open gripper\np - rotate gripper\nb - return to previous menu\n";
                        while(cmd != 'b'){
                            cmd = getch();
                            switch(cmd){
                                case 'i':
                                    Ard_char[1] = 'c';
                                    SendGripperSerial(hComm, Ard_char);
                                    break;
                                case 'o':
                                    Ard_char[1] = 'o';
                                    SendGripperSerial(hComm, Ard_char);
                                    break;
                                case 'p':
                                    Ard_char[1] = 'd';
                                    cout << "Rotation: ";
                                    cin >> attnStringBuf;
                                    copy(begin(attnStringBuf), begin(attnStringBuf)+4, begin(Ard_char)+3); // Write the first 4 char into the serial array
                                    SendGripperSerial(hComm, Ard_char);
                                    fill(begin(Ard_char)+3, begin(Ard_char)+7, ' '); // Reset the ending number with space char
                                    break;
                                case 'r':
                                    Ard_char[1] = 'r';
                                    SendGripperSerial(hComm, Ard_char);
                                    break;
                            }                        
                        }
                        cout << "Gripper testing terminated.\n";
                        break;
                    // case 'l':   // Linear rail motions
                    //     cout << "0 to 3 - home linear rail individually\n4 - home all 4 linear rails automatically\nb - Back to previous menu\nany other keys - stop the linear rail from current motion\n";
                    //     while(cmd != 'b'){
                    //         cin >> cmd;
                    //         if('/' < cmd && cmd < 52){ // homing individually
                    //             int id = cmd - 48;
                    //             cout << "Homing linear rail #"<< cmd <<".\n";
                    //             HomeLinearRail(id);
                    //         }
                    //         else if(cmd == 52){ // homing 4 all tgt
                    //             for (int i = 0; i < 4 ; i++){
                    //                 HomeLinearRail(i);
                    //             }
                    //             cout << "All linear rails are homed.\n";
                    //         }
                    //     }
                    //     cout << "Linear rail homing terminated\n";
                    //     break; 
                    case 'u':   // Update in1[] and offset[] from csv file
                        ifstream file ("lastPos.txt"); //"lastPos.txt" or "currentPos.csv"
                        string temp;
                        int count = 0;
                        if(file.is_open()){
                            try{
                                while (file >> temp){
                                    if(count > 5) { break; } //{ railOffset = stod(temp); break; } // reading the rail offset, then break while loop
                                    robot.in[count++] = stod(temp); // convert string to double stod()
                                }
                                cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
                            }
                            catch(int e){ cout << "Check if currentPos.csv matches the in1 input no." << endl; }
                            // do{
                            //     cout << "Current linear rail offset: ";
                            //     cin >> railOffset; // do we need other constraits? ie 0 <= railOffset < 2
                            // }while(!cin.good() || railOffset>2);
                            
                            robot.PoseToLength(robot.in, robot.out, railOffset); //pose_to_length(in1, out1, railOffset);
                            for (int n = 0; n < nodeList.size(); n++){
                                int32_t step = robot.ToMotorCmd(n, robot.out[n]);
                                nodeList[n]->Motion.PosnMeasured.Refresh();
                                nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value() + step);
                            }
                            cout << "Updating motor counts completed" << endl;
                            cout << "Current coordinates: "; robot.PrintIn();
                            cout << "Motor internal counts: ";
                            for (int id = 0; id < nodeList.size(); id++){
                                nodeList[id]->Motion.PosnMeasured.Refresh();
                                cout << (double) nodeList[id]->Motion.PosnMeasured << "\t";
                            }
                            cout << endl;
                            cout << "Linear rail offset: " << railOffset << endl;
                        }
                        break;

                }
            } while(cmd != 'n');
        }
        catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
            printf("ERROR: Motor command failed.\n");  
            printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return -3;
        }
        
        // Set linear rail brakes as usually enabled
        // myMgr->Ports(2).BrakeControl.BrakeSetting(0, BRAKE_PREVENT_MOTION);

        cout << "Choose from menu for cable robot motion:\nt - Read from \"bricks.csv\" file for brick positions\nl - Loop through set num of bricks\nb - Loop and wait for Button input\nm - Manual input using w,a,s,d,r,f,v,g\np - Run Point-to-point trajectory\nq - Request current torQue readings\ni - Info: show menu\nn - Prepare to disable motors and exit programme\nr - Return to previous section" << endl;
        do {
            bool waitBtn = false;
            cin >> cmd;
            switch (cmd){
                case 'i':    // Show menu
                    cout << "Choose from menu for cable robot motion:\nt - Read from \"bricks.csv\" file for brick positions\nl - Loop through set num of bricks\nb - Loop and wait for Button input\nm - Manual input using w,a,s,d,r,f,v,g\np - Run Point-to-point trajectory\nq - Request current torQue readings\ni - Info: show menu\nn - Prepare to disable motors and exit programme\nr - Return to previous section" << endl;
                    break;
                case 't':   // Read brick file, plan trajectory
                case 'T':
                    if(!ReadBricksFile()){ continue; } // Read "bricks.csv"
                    RunBricksTraj(hComm, robot, Ard_char, 0, true);
                    break;
                case 'm':   // Manual wasdrf
                case 'M':
                    cout << "Press 'q' to quit manual input anytime.\n'h' for Homing.\n'x' to adjust increment step size.\n";
                    // nodeList[8]->Port.BrakeControl.BrakeSetting(0, BRAKE_ALLOW_MOTION); // disable enable brake
                    while(cmd != 'q' && cmd != 'Q'){
                        cmd = getch();
                        switch(cmd){
                            case 'I':
                            case 'i':
                                Ard_char[1] = 'c';
                                SendGripperSerial(hComm, Ard_char);
                                continue;
                            case 'O':
                            case 'o':
                                Ard_char[1] = 'o';
                                SendGripperSerial(hComm, Ard_char);
                                continue;
                            case 'P':
                            case 'p':
                                Ard_char[1] = 'd';
                                cout << "Rotation: ";
                                cin >> attnStringBuf;
                                copy(begin(attnStringBuf), begin(attnStringBuf)+4, begin(Ard_char)+3); // Write the first 4 char into the serial array
                                SendGripperSerial(hComm, Ard_char);
                                fill(begin(Ard_char)+3, begin(Ard_char)+7, ' '); // Reset the ending number with space char
                                continue;
                            case 'W':
                            case 'w':
                                robot.in[1] += step;
                                break;
                            case 'S':
                            case 's':
                                robot.in[1] -= step;
                                break;
                            case 'A':
                            case 'a':
                                robot.in[0] -= step;
                                break;
                            case 'D':
                            case 'd':
                                robot.in[0] += step;
                                break;
                            case 'R':
                            case 'r':
                                robot.in[2] += step;
                                break;
                            case 'F':
                            case 'f':
                                robot.in[2] -= step;
                                break;
                            case 'H':
                            case 'h':
                                cout << "Homing...\n"; 
                                TrjHome(robot);
                                break;
                            case 'X':
                            case 'x':
                                cout << "Current step size: " << step << "m. Please enter new step size: ";
                                cin >> step;
                                if (!cin.good()){ cout << "Invalid input!"; }
                                else if (abs(step) > 0.1){ cout << "Warning! Step size is too large, may cause vigorious motions.";}
                                else { cout << "New step size: " << step << endl; break; }
                                cout << " Step size is now set to 0.01m.\n";
                                step = 0.01;
                                break;
                            case 'C':
                            case 'c':
                                cout << "Are you sure to clear warnings from triggered linear rail limit switch? Type \"CLEAR\" to confirm.\n";
                                {
                                    string reply;
                                    cin >> reply;
                                    if (reply == "CLEAR"){ limitType = 'C'; cout << "Warning dismissed.\n"; }
                                    else { cout << "Warning is not cleared.\n"; }
                                }
                                continue;
                        }
                        robot.PrintIn();
                        if(robot.CheckLimits()){
                            robot.PoseToLength(robot.in, robot.out, railOffset);
                            robot.PrintOut();
                            SendMotorGrp(robot);
                            
                            Sleep(step*1000);
                        }
                        else{
                            cout << "WARNING: Intended position out of bound!\n";
                            switch(cmd){
                                case 'W':
                                case 'w':
                                    robot.in[1] -= step;
                                    break;
                                case 'S':
                                case 's':
                                    robot.in[1] += step;
                                    break;
                                case 'A':
                                case 'a':
                                    robot.in[0] += step;
                                    break;
                                case 'D':
                                case 'd':
                                    robot.in[0] -= step;
                                    break;
                                case 'R':
                                case 'r':
                                    robot.in[2] -= step;
                                    break;
                                case 'F':
                                case 'f':
                                    robot.in[2] += step;
                                    break;
                            }
                        }
                    }
                    // nodeList[8]->Port.BrakeControl.BrakeSetting(0, BRAKE_PREVENT_MOTION); // enable brake afterwards
                    cout << "Quit manual control\n";
                    break;
                case 'b':   // loop through set no. of bricks and wait for user Button input to continue
                case 'B':
                    waitBtn = true;
                case 'l':   // Read brick file, loop through a set no. of bricks
                case 'L':
                    {int loopNum = 6; // Define the no. of bricks to loop here!!!
                    // Read input file for traj-gen
                    quitType = 'r';
                    if(!ReadBricksFile()){ continue; } // Read "bricks.csv"
                    cout << "Bricks to loop: " << loopNum << endl;
                    if(brickPos.size()<loopNum){ cout << "Warning! Defined brick file is not long enough for looping.\n"; break; }
                    // int listOffset = brickPos.size() - loopNum;
                    int listOffset = 683 - loopNum; //683 is total no. of brick in the current file
                    brickPos.erase(brickPos.begin(), brickPos.end()-loopNum); // Only need the last elements
                    while(quitType != 'f' && quitType != 'F'){ // if not running the final loop.....
                        // always reverse from a complete built, then rebuild it
                        ReverseBricksTraj(robot, listOffset, true);
                        if(quitType == 'q' || quitType == 'Q' || quitType == 'e'){ break; }
                        RunBricksTraj(hComm, robot, Ard_char, listOffset, true, waitBtn);
                        if(quitType == 'q' || quitType == 'Q' || quitType == 'e'){ break; }
                        loopCount += 1;
                        // now = time(0); fn = localtime(&now); if(fn->tm_hour >= SleepTime) { break; } // quit loop after sleep time
                        if(quitType != 'f' && !waitBtn){
                            cout << "Taking a 3 minute rest~ ^O^\n\n";
                            Sleep(1000*180);
                        }
                    }
                    cout << "Quit looping trajectory.\n";}
                    break;
                case 'p':   // Typical point to point trajectory
                case 'P':
                    RunTrajPoints(robot);
                    break;
                case 'q':   // Request current torque readings
                case 'Q':
                    for(int i = 0; i < nodeList.size(); i++){                    
                        nodeList[i]->Motion.TrqMeasured.Refresh();
                        cout << "[" << i << "]: " << nodeList[i]->Motion.TrqMeasured.Value() << endl;
                    }   
                    break;
            }
        } while(cmd != 'n' && cmd != 'r');
    } while(cmd != 'n' && quitType != 'e');

    //// Safe system shut down, safe last pos and emegency shut down
    // Saving last position before quiting programme
    cout << "Saving last position...\n";
    ofstream myfile;
    myfile.open ("lastPos.txt");
    myfile << robot.in[0] << " " << robot.in[1] << " " << robot.in[2] << " " << robot.in[3] << " " << robot.in[4] << " " << robot.in[5] << endl;
    myfile << railOffset << endl;
    for(INode* n : nodeList){
        n->Motion.PosnMeasured.Refresh();
        myfile << n->Motion.PosnMeasured.Value() << " ";
    }
    myfile.close();
    tm *fn; time_t now = time(0); fn = localtime(&now);
    myfile.open("log.txt", ios::app);
    myfile <<  fn->tm_mon +1 << "/" << fn->tm_mday << ": " << loopCount << " loop runs. ABB error hard-code call: " << abbCount << endl;
    myfile.close();
    
    //// List of what-if-s??
    
    {   // Send 'f' signal to bluetooth gripper for shutting down
        Ard_char[1] = 'f';
        SendGripperSerial(hComm, Ard_char);
        CloseHandle(hComm); //Close the Serial Port
    }

    for(int i = 0; i < nodeList.size(); i++){ //Disable Nodes
        nodeList[i]->EnableReq(false);
    }
    nodeList.clear();
    myMgr->PortsClose(); // Close down the ports
    cout << "Teknic motors are disabled\n";
    return 1;
}

bool SetSerialParams(HANDLE hComm){
    // Set parameters for serial port
    DCB dcbSerialParams = { 0 }; // Initializing DCB structure
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    bool Status = GetCommState(hComm, &dcbSerialParams); // retreives  the current settings
    if (Status == false){ cout << "Error in GetCommState()\n"; return false; }

    dcbSerialParams.BaudRate = CBR_57600;// Setting BaudRate
    dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
    dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
    dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None

    SetCommState(hComm, &dcbSerialParams);
    if (Status == false){ cout << "Error! in Setting DCB Structure\n"; return false; }
    else{
        printf("   Setting DCB Structure Successful\n");
        printf("       Baudrate = %d\n", dcbSerialParams.BaudRate);
        printf("       ByteSize = %d\n", dcbSerialParams.ByteSize);
        printf("       StopBits = %d\n", dcbSerialParams.StopBits);
        printf("       Parity   = %d\n", dcbSerialParams.Parity);
        cout << endl;
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(hComm, &timeouts) == FALSE){ cout << "Error! in Setting Time Outs\n"; return false; }
    // Set recieve mask                
    if (!(bool)SetCommMask(hComm, EV_RXCHAR)){ cout << "Error! in Setting CommMask\n"; }
    
    return true;
}

void ReadGripperSerial(HANDLE hComm){
    DWORD dwEventMask, BytesRead;
    int i{0};
    char tmp, msg[256];

    bool Status = WaitCommEvent(hComm, &dwEventMask, NULL); // wait till brick is ready from ABB
    if (Status == false){ cout << "Error in setting WaitCommEvent()\n";} //quitType = 'q'; return; }
    else {
        do{
            ReadFile(hComm, &tmp, sizeof(tmp), &BytesRead, NULL);
            msg[i] = tmp;
            i++;
        }while (BytesRead>0);        
    }
    
    for(int j=0; j< i-1; j++){
        cout << msg[j];
    } 
}

void SendGripperSerial(HANDLE hComm, unsigned char* Ard_char){
    DWORD dNoOfBytesWritten = 0;
    if (!(bool)WriteFile(hComm, Ard_char, 8, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; }    
    ReadGripperSerial(hComm);
    ReadGripperSerial(hComm); // Repeat to make sure the feedback is received
}

int CheckMotorNetwork() {
    SysManager* myMgr = SysManager::Instance();

    sFnd::SysManager::FindComHubPorts(comHubPorts);

    cout << "Found " <<comHubPorts.size() << " SC Hubs\n";
    for (portCount = 0; portCount < comHubPorts.size(); portCount++) {
        myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());
    }
    if (portCount < 0) {
        cout << "Unable to locate SC hub port\n";
        return -1;
    }
    if(portCount==0) { return -1; } // do we need this?
    
    myMgr->PortsOpen(portCount);
    for (int i = 0; i < portCount; i++) { // check no. of nodes in each ports
        IPort &myPort = myMgr->Ports(i);
        // myPort.BrakeControl.BrakeSetting(0, BRAKE_AUTOCONTROL); // do we need this?
        // myPort.BrakeControl.BrakeSetting(1, BRAKE_AUTOCONTROL);
        printf(" Port[%d]: state=%d, nodes=%d\n", myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
    
        for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {
            INode &theNode = myPort.Nodes(iNode);
            theNode.EnableReq(false); //Ensure Node is disabled before loading config file
            myMgr->Delay(200);

            printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
            printf("            userID: %s\n", theNode.Info.UserID.Value());
            printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
            printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
            printf("             Model: %s\n", theNode.Info.Model.Value());

            theNode.Status.AlertsClear();               //Clear Alerts on node 
            theNode.Motion.NodeStopClear();             //Clear Nodestops on Node               
            theNode.EnableReq(true);                    //Enable node 
            theNode.Motion.PosnMeasured.AutoRefresh(true);
            theNode.Motion.TrqMeasured.AutoRefresh(true);
            printf("Node %d enabled. ", iNode);

            theNode.AccUnit(INode::RPM_PER_SEC);        //Set the units for Acceleration to RPM/SEC
            theNode.VelUnit(INode::RPM);                //Set the units for Velocity to RPM
            theNode.Motion.AccLimit = 40000;           //100000 Set Acceleration Limit (RPM/Sec)
            theNode.Motion.NodeStopDecelLim = 5000;
            theNode.Motion.VelLimit = 3000;             //700 Set Velocity Limit (RPM)
            theNode.Info.Ex.Parameter(98, 1);           //enable interrupting move
            theNode.Motion.Adv.TriggerGroup(1);         //Set all nodes trigger group num. as 1
            cout << "AccLimit and VelLimit set." << endl;

            nodeList.push_back(&theNode);               // add node to list

            double timeout = myMgr->TimeStampMsec() + 2000; //TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
            //This will loop checking on the Real time values of the node's Ready status
            while (!theNode.Motion.IsReady()) {
                if (myMgr->TimeStampMsec() > timeout) {
                    printf("Error: Timed out waiting for Node %d to enable\n", iNode);
                    return -2;
                }
            }
        }
    }
    return 0;
}

int RaiseRailTo(CDPR &r, double target){ // !!! Define velocity limit !!!
    cout << "RAil target: " << target << endl;
    if (target < 0 || target > 1.25) { cout << "WARNING! Intended rail offset is out of bound!\n"; return -2; }
    nodeList[8]->Port.BrakeControl.BrakeSetting(0, BRAKE_ALLOW_MOTION); // disable brake before motion
    double velLmt = 0.005; // meters per sec
    double dura = abs(target - railOffset)/velLmt*1000;// *1000 to change unit to ms
    if(dura <= 200){ return 0; } // Don't run traj for incorrect timing 
    double a, b, c; // coefficients for cubic spline trajectory
    cout << "\nATTENTION: Raise rail function is called. Estimated time: " << dura << "\n";
    
    // Solve coefficients of equations for cubic
    a = railOffset;
    b = 3 / (dura * dura) * (target - railOffset); 
    c = -2 / (dura * dura * dura) * (target - railOffset);
    
    // Run the trajectory till the given time is up
    double t = 0;
    auto start = chrono::steady_clock::now();
    long dur = 0;
    while (t <= dura){        
        // CUBIC equation, per time step pose
        railOffset = a + b * t * t + c * t * t * t; // update new rail offset

        // get absolute cable lengths and rail positions in meters
        r.PoseToLength(r.in, r.out, railOffset);
        r.PrintOut();

        SendMotorGrp(r, false, true);

        dur = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now()-start).count();
        double dif = MILLIS_TO_NEXT_FRAME - dur - 1;
        if(dif > 0) { Sleep(dif);}
        t += MILLIS_TO_NEXT_FRAME;
        start = chrono::steady_clock::now(); // refresh the ending time in loop

        if(kbhit()){ // Emergency quit during trajectory control
            cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nf - Finish this loop of motion\nr - Resume trajectory\n";
            cin >> quitType;
            if(quitType == 'q' || quitType == 'Q' || quitType == 'e'){
                cout << "Trajectory emergency quit\n";
                nodeList[8]->Port.BrakeControl.BrakeSetting(0, BRAKE_PREVENT_MOTION); // enable brake after
                return -1;
            }
        }
        if(limitType != 'C'){
            cout << "WARNING! Linear rail limits triggered. Please quit the programme and check the system.\n";
            return -3;
        }
    }
    nodeList[8]->Port.BrakeControl.BrakeSetting(0, BRAKE_PREVENT_MOTION); // enable brake after motion */
    return 0;
}

int RunParaBlend(CDPR &r, double point[7], bool showAttention){
    float vMax[6] = {.4, .4, .4, 0.8, 0.8, 0.8}; // m/s, define the maximum velocity for each DoF
    float aMax[6] = {80, 80, 80, 10, 10, 10}; // m/s^2, define the maximum acceleration for each DoF
    static double a[6], b[6], c[6], d[6], e[6], f[6], g[6], tb[6]; // trajectory coefficients
    static double sQ[6], Q[6], o[6];
    static double dura;
    double unitV = sqrt(pow(point[0]-r.in[0],2)+pow(point[1]-r.in[1],2)+pow(point[2]-r.in[2],2)); // the root to divide by to get unit vector
    dura = point[6]; // Save the new dura for every new point
    cout << "Will run for " << dura << "ms...\n";
    if(dura <= 200){ return 0; } // Don't run traj for incorrect timing 

    // Solve parabolic blend coefficients for each DoF
    for(int i = 0; i < 6; i++){
        sQ[i] = r.in[i]; // start point, from current position
        Q[i] = point[i]; // end point, from goal position
        vMax[i] /= 1000; // change the velocity unit to meter per ms
        aMax[i] /= 1000000; // change the unit to meter per ms square
        tb[i] = i<3? dura-unitV/vMax[i] : dura-abs(Q[i]-sQ[i])/vMax[i];
        if(tb[i] < 0) {
            cout << "WARNING: Intended trajectory exceeds velocity limit in DoF "<< i << ".\n";
            return -1;
        }
        else if (tb[i] > dura / 2){
            if (showAttention){ cout << "ATTENTION: Trajectory for DoF " << i << " will be in cubic form.\n"; }
            tb[i] = dura / 2;
            vMax[i] = 2 * (Q[i] - sQ[i]) / dura;
        }
        else if(i<3){ vMax[i] = vMax[i] * (Q[i] - sQ[i]) / unitV; } // vMax in x,y,z accordingly
        else if(Q[i]<sQ[i]){ vMax[i] *= -1; } //Fix velocity direction for rotation
        o[i] = vMax[i] / 2 / tb[i]; // times 2 to get acceleration
        if(abs(o[i]*2) > aMax[i]){
            cout << "WARNING: Intended trajectory acceleration <" << abs(o[i]*2) << "> exceeds limit in DoF "<< i << ".\n";
            return -1;
        }
        
        // solve coefficients of equations for parabolic
        a[i] = sQ[i];
        b[i] = o[i];

        c[i] = sQ[i] - vMax[i] * tb[i] / 2;
        d[i] = vMax[i];

        e[i] = Q[i] - o[i] * dura * dura;
        f[i] = 2 * o[i] * dura;
        g[i] = -o[i];
    }
    
    // Run the trajectory till the given time is up
    double t = 0;
    while (t <= point[6]){
        auto start = chrono::steady_clock::now();
        long dur = 0;
        
        // PARABOLIC BLEND equation, per time step pose
        for (int j = 0; j < 6; j++){
            if (t <= tb[j]){
                r.in[j] = a[j] + b[j] * t * t;
            }
            else if(t <= point[6]-tb[j]){
                r.in[j] = c[j] + d[j] * t;
            }
            else{
                r.in[j] = e[j] + f[j] * t + g[j] * t * t;
            }
        }
        // get absolute cable lengths in meters
        // cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        r.PoseToLength(r.in, r.out, railOffset);
        // cout << "OUT: "<<  out1[0] << "\t" << out1[1] << "\t" << out1[2] << "\t" << out1[3] << "\t" <<  out1[4] << "\t" << out1[5] << "\t" << out1[6] << "\t" << out1[7] << endl;
        
        SendMotorGrp(r);
        if(quitType == 'e'){ // Motor error message?
            cout << "WARNING! Motor error message received. System will now shut now.\n";
            return -4;
        }

        // Write to traking file
        ofstream myfile;
        myfile.open ("traking.txt");
        myfile << r.in[0] << " " << r.in[1] << " " << r.in[2] << " " << r.in[3] << " " << r.in[4] << " " << r.in[5] << endl;
        myfile << railOffset << endl;
        myfile.close();

        auto end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        
        double dif = MILLIS_TO_NEXT_FRAME - dur - 1;
        if(dif > 0) { Sleep(dif);}
        t += MILLIS_TO_NEXT_FRAME;

        if(kbhit()){ // Emergency quit during trajectory control
            cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nf - Finish this loop of motion\nr - Resume trajectory\n";
            cin >> quitType;
            if(quitType == 'q' || quitType == 'Q'){
                cout << "Trajectory emergency quit\n";
                return -2;
            }
        }
        if(limitType != 'C'){
            cout << "WARNING! Linear rail limits triggered. Please quit the programme and check the system.\n";
            return -3;
        }
    }
    return 0;
}

// double ScaleRailLvl(double brickLvl){
//     double output = brickLvl * 0.8;
//     if (output > 1.2){ return 1.2; } // Upper limit of linear rail
//     else if (output < 0) { return 0; } // lower limit of linear rail
//     return output;
// }

//void RunBricksTraj(const dynamixel::GroupSyncRead &groupSyncRead, int listOffset, bool showAttention, bool waitBtn){
void RunBricksTraj(HANDLE hComm, CDPR &r, unsigned char* Ard_char, int listOffset, bool showAttention, bool waitBtn){
    double brickPickUp[7] = {9.9, 6.72, -2.97, 0, 0, 0, 10}; // !!!! Define the brick pick up point !!!!, the last digit is a dummy number for time duration.
    double safePt[3] = {8.3, 6.72, -2.7}; // a safe area near to the arm // 0.21 safe height from ABB
    double goalPos[7] = {2, 2, 1, 0, 0, 0, 10}; // updated according to brick position
    double velLmt = 0.14; // meters per second //////// -65 deg for pick up //////////9.88 6.71 -2.76 safe height after pick up
    double safeT = 1600; // in ms, time to raise to safety height
    double safeH = 0.12; // meter, safety height from building brick level
    double currentBrkLvl = railOffset; // meter, check if the rail offset is the same as target BrkLvl
    double dura = 0;
    unsigned char tmp, msg[256];
    DWORD dwEventMask, BytesRead, dNoOfBytesWritten = 0;
    bool Status;

    // Go through the given bricks
    for (int i = 0; i < brickPos.size(); i++) {
        ofstream myfile; int timeout_i = 0;

        // // Check if rails need to be raised
        // if(ScaleRailLvl(brickPos[i][2] - 0.04) != currentBrkLvl){
        //     currentBrkLvl = ScaleRailLvl(brickPos[i][2] - 0.0404); // Offset one brick height from building levei, ie 0.0404m
        //     if(RaiseRailTo(currentBrkLvl) < 0) { cout << "Trajectory aborted.\n"; return; } // raise rail to the building brick level
        // }

        // Wait for button input before builing a brick
        if(waitBtn && quitType != 'f'){
            cout << "...Waiting for button input...\n";
            myfile.open ("indexBrk.txt");
            myfile << -2; // attraction screen
            myfile.close();
            do{
                nodeList[0]->Status.RT.Refresh(); // Refresh again to check if button is pushed
                if(kbhit()){ // Emergency quit during trajectory control
                    cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nf - Finish this loop of motion\nr - Resume trajectory\n";
                    cin >> quitType; break;
                }
                time_t now = time(0); tm *fn = localtime(&now); if(fn->tm_hour >= SleepTime) { quitType = 'f'; break; } // Quit button loop directly after sleep time
            }while(!nodeList[0]->Status.RT.Value().cpm.InA);
        }

        // Update index file for grasshopper display
        myfile.open ("indexBrk.txt");
        myfile << i + 1 + listOffset;
        myfile.close();

        // Go pick up a brick
        if(showAttention) { cout << "Picking up a brick\n"; }
        //// ... inform brick station ... ////
        Ard_char[1] = 'd'; Ard_char[3] = '-'; Ard_char[4] = '6'; Ard_char[5] = '3'; Ard_char[6] = ' ';
        SendGripperSerial(hComm, Ard_char); // Rotate gripper // -63 deg at brick pick-up station
        Ard_char[3] = ' '; Ard_char[4] = ' '; // Reset the ending number with space char
        Sleep(50); // short break between bluetooth communication
        Ard_char[1] = 'o';
        SendGripperSerial(hComm, Ard_char); // Open gripper
        goalPos[6] = sqrt(pow(brickPickUp[0]-r.in[0],2)+pow(brickPickUp[1]-r.in[1],2)+pow(brickPickUp[2]+0.21-r.in[2],2))/velLmt*1000; // calculate time // 0.21 is height above brick pickup
        copy(brickPickUp, brickPickUp+3, begin(goalPos));
        goalPos[2] += 0.21; // 0.21 safe height from ABB
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // Go to safe height above pick up
        Sleep(100);
        //// ... wait for brick to be available ... ////
        
        // fall and pick up brick
        brickPickUp[6] = safeT*1.2;
        if(RunParaBlend(r, brickPickUp) < 0) { cout << "Trajectory aborted.\n"; return; } // Go pick up
        Sleep(600); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
        Ard_char[1] = 'c';
        SendGripperSerial(hComm, Ard_char); // Close gripper
        Sleep(800); // wait for grippper to close
        //// ... wait for brick released from station, also 5th pole is at safe position ... ////

        // Go to building level
        if(showAttention) { cout << "Raising brick from station\n"; }
        copy(brickPickUp, brickPickUp+3, begin(goalPos));
        goalPos[2] += 0.14;
        goalPos[6] = safeT;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise the brick from robot arm
        if(showAttention) { cout << "Going to building level\n"; }
        Ard_char[1] = 'd';
        string s = to_string((int)brickPos[i][3]);
        for(int i=0; i<min(s.size(),4); i++){ Ard_char[i+3]=s[i]; }
        SendGripperSerial(hComm, Ard_char); // Rotate gripper according to brickPos
        copy(safePt, safePt+3, begin(goalPos)); // safe point
        goalPos[6] = sqrt(pow(safePt[0]-r.in[0],2)+pow(safePt[1]-r.in[1],2)+pow(safePt[2]-r.in[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point
        goalPos[2] = brickPos[i][2] + r.EEOffset() + safeH; // brick level with safe height
        goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2)+pow(goalPos[2]-r.in[2],2))/velLmt*1000;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise the brick to building level
        
        // Go to brick placing position
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];
        goalPos[1] = brickPos[i][1];
        goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2))/velLmt*1000;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }

        // Place brick
        if(showAttention) { cout << "Placing brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        Ard_char[1] = 'o';
        SendGripperSerial(hComm, Ard_char); // Open gripper
        Sleep(200); //Wait a while after placing brick
        
        // Rise and leave building area, stand by for next brick pick up
        if(showAttention) { cout << "Going to stand by position\n"; }
        goalPos[2] += safeH;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
        Ard_char[1] = 'd'; Ard_char[3] = '3'; Ard_char[4] = '0'; Ard_char[5] = ' '; Ard_char[6] = ' ';
        SendGripperSerial(hComm, Ard_char); // Rotate gripper // 30 deg at brick pick-up station
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y,z position
        goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2)+pow(goalPos[2]-r.in[2],2))/velLmt*1000;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        r.PrintIn();
        cout << railOffset << endl;
        cout << "----------Completed brick #" << i + 1 + listOffset <<"----------" << endl;
    }
    Ard_char[1] = 'r';
    SendGripperSerial(hComm, Ard_char); // Release gripper
    copy(r.home, r.home+3, begin(goalPos));
    goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2)+pow(goalPos[2]-r.in[2],2))/velLmt*1000;
    if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // home after building all
    cout << "Photo time~~\n"; Sleep(3000);
}

// void ReverseBricksTraj(const dynamixel::GroupSyncRead &groupSyncRead, int listOffset, bool showAttention){
void ReverseBricksTraj(CDPR &r, int listOffset, bool showAttention){
    double brickDropOff[7] = {0.72, 2.14, 1.9, 0, 0, 0, 10}; // !!!! Define the brick drop off point !!!!, the last digit is a dummy number for time duration. // rotation 165 for drop off
    double safePt[3] = {1.6, 1.8, 2.05}; // a safe area near the drop off
    double goalPos[7] = {2, 2, 1, 0, 0, 0, 10}; // updated according to brick position
    double velLmt = 0.27; // meters per second
    double safeT = 1100; // in ms, time to raise to safety height //1200 ms
    double safeH = 0.08; // meter, safety height from building brick level
    double currentBrkLvl = railOffset; // meter, check if the rail offset is the same as target BrkLvl
    double dura = 0;

    ofstream myfile;
    myfile.open ("indexBrk.txt");
    myfile << -1; // -1 for disassembly
    myfile.close();
    
    // Go through the given bricks
    for (int i = brickPos.size()-1; i > -1; i--) {
        // // Check if rails need to be raised
        // if(ScaleRailLvl(brickPos[i][2] - 0.04) != currentBrkLvl){
        //     currentBrkLvl = ScaleRailLvl(brickPos[i][2] - 0.04); // Offset one brick height from building levei, ie 0.04m
        //     if(RaiseRailTo(currentBrkLvl) < 0) { cout << "Trajectory aborted.\n"; return; } // raise rail to the building brick level
        // }

        // Go to building level
        if(showAttention) { cout << "Going to building level\n"; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        copy(r.in, r.in+2, begin(goalPos));
        goalPos[2] = brickPos[i][2] + r.EEOffset() + safeH; // brick level
        goalPos[6] = sqrt(pow(goalPos[2]-r.in[2],2))/velLmt*1000 + 800; // calculate time + hardcode time to smooth out sudden jump
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise from current pos to builing level
        
        // Go to brick placing position
        // rotationG = brickPos[i][3] * 11.26666;; // conversion from angle to motor command
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];                       
        goalPos[1] = brickPos[i][1];
        goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2))/velLmt*1000;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        
        // Retrieve brick
        if(showAttention) { cout << "Retrieving brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; return; }
        Sleep(1500); // Wait for gripper to close

        // Rise and leave building area, stand by for next brick pick up
        if(showAttention) { cout << "Going to drop off position\n"; }
        goalPos[2] += safeH;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, neutralRot, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y position
        goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2)+pow(goalPos[2]-r.in[2],2))/velLmt*1000;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        // Dropping off a brick
        if(showAttention) { cout << "Dropping off the brick\n"; }
        brickDropOff[6] = sqrt(pow(brickDropOff[0]-r.in[0],2)+pow(brickDropOff[1]-r.in[1],2)+pow(brickDropOff[2]-r.in[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(r, brickDropOff) < 0) { cout << "Trajectory aborted.\n"; return; }
        brickDropOff[6] = 800; // Safe time for dropping
        brickDropOff[2] -= 0.02; // go down a little more before real drop
        if(RunParaBlend(r, brickDropOff) < 0) { cout << "Trajectory aborted.\n"; return; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        Sleep(500); // wait a little after dropping brick
        brickDropOff[2] += 0.02; // go up a little more after real drop
        if(RunParaBlend(r, brickDropOff) < 0) { cout << "Trajectory aborted.\n"; return; }
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe point
        goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2)+pow(goalPos[2]-r.in[2],2))/velLmt*1000;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        r.PrintIn();
        cout << "----------Retrieved brick #" << i + 1 + listOffset <<"----------" << endl;
    }
    // packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, 0, &dxl_error);
    copy(r.home, r.home+3, begin(goalPos));
    goalPos[6] = sqrt(pow(goalPos[0]-r.in[0],2)+pow(goalPos[1]-r.in[1],2)+pow(goalPos[2]-r.in[2],2))/velLmt*1000;
    if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // home after retrieving all
}

void RunTrajPoints(CDPR &r){
    // Read raw points from traj.csv
    ifstream file ("traj.csv");
    vector<double> row;
    string line, word, temp;

    brickPos.clear();
    if(file.is_open()){
        while (getline(file, line)){
            row.clear();
            stringstream s(line);
            while (s >> word){
                row.push_back(stod(word)); // convert string to double stod()
            }
            brickPos.push_back(row);
        }
        cout << "Completed reading traj input file" << endl;
    }
    else{ cout << "Failed to read input file. Please check \"traj.csv\" file." << endl; return;
    }
    
    // Go through the given points
    double goalPos[7] {};
    for (int i = 0; i < brickPos.size(); i++) {
        copy(begin(brickPos[i]), end(brickPos[i]), begin(goalPos));
        // cout << "i: " << i << endl;
        // cout << "brickPos("<< brickPos[i].size()<<"): "<< brickPos[i][0]<<","<< brickPos[i][1]<<","<< brickPos[i][2]<<","<< brickPos[i][3]<<","<< brickPos[i][4]<<","<< brickPos[i][5]<<","<< brickPos[i][6]<<endl;
        if(RunParaBlend(r, goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }              
        r.PrintIn();
        cout << railOffset << endl;
        cout << "----------Completed brick #" << i + 1 <<"----------" << endl;
    }
}

void SendMotorCmd(CDPR &r, int n){
    ofstream myfile;
    // convert to absolute cable length command
    try{
        // int32_t step = ToMotorCmd(n, out1[n]);
        int32_t step = r.ToMotorCmd(n, r.out[n]);
        // nodeList[n]->Motion.MoveWentDone();
        // nodeList[n]->Motion.MovePosnStart(step, true, true); // absolute position
        nodeList[n]->Motion.Adv.MovePosnStart(step, true, true); // absolute position, wait for trigger
        // float trq = nodeList[n]->Motion.TrqMeasured.Value();
        // if(abs(trq)>r.AbsTorqLmt()){
        //     myfile.open("log.txt", ios::app);
        //     myfile << "Motor [" << n << "] exceeds torque limit: " << trq;
        //     myfile.close();
        //     quitType = 'e';
        //     cout << "ATTENTION: Motor [" << n << "] exceeds torque limit: " << trq << endl;
        // }
        // nodeList[n]->Motion.Adv.TriggerGroup(1);

        if (nodeList[n]->Status.Alerts.Value().isInAlert()) {
            myfile.open("log.txt", ios::app);
            myfile << "Alert from Motor [" << n << "]: "<< nodeList[n]->Status.Alerts.Value().bits <<"\n";
            myfile.close();
        }
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        cout << "\nERROR: Motor [" << n << "] command failed.\n";  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        quitType = 'e';
        tm *fn; time_t now = time(0); fn = localtime(&now);
        myfile.open("log.txt", ios::app);
        myfile << "\nERROR: Motor [" << n << "] command failed. " << fn->tm_hour << ":"<< fn->tm_min << ":" << fn->tm_sec <<"\n";
        myfile << "Caught error: addr="<< (int) theErr.TheAddr<<", err="<<hex<<theErr.ErrorCode <<"\nmsg="<<theErr.ErrorMsg<<"\n";
        myfile.close();
    }
}

void SendMotorTrq(CDPR &r, int n){
    nodeList[n]->Motion.TrqCommanded.Refresh();
    float currentTorque = nodeList[n]->Motion.TrqCommanded.Value();

    if(currentTorque > targetTorque){ nodeList[n]->Motion.MoveVelStart(-300); }
    else if (currentTorque < targetTorque - 1.8){ nodeList[n]->Motion.MoveVelStart(150); cout << "Too much torque!!\n";}
    else{ nodeList[n]->Motion.MoveVelStart(-10);}
    printf("Node[%d], current torque: %f\n", n, currentTorque);
}

void SendMotorGrp(CDPR &r, bool IsTorque, bool IsLinearRail){
    SysManager* myMgr = SysManager::Instance();
    IPort &myPort = myMgr->Ports(0); // only if one scHub is used
    void (*func)(CDPR&, int){ SendMotorCmd };
    if(IsTorque){ func = SendMotorTrq; }
    int n = IsLinearRail? 4 : 0; // offset in nodeList
    
    thread nodeThreads[8];
    for(int i = 0; i < r.NodeNum(); i++){
        nodeThreads[i] = thread((*func), r, i + n); 
    }
    for(int i = 0; i < r.NodeNum(); i++){
        nodeThreads[i].join();
    }
    // myPort.Adv.TriggerMovesInGroup(1);
    if (quitType!='e'){ myPort.Adv.TriggerMovesInGroup(1); } // Only move all if no error is caugth
}

void TrjHome(CDPR &r){// !!! Define the task space velocity limit for homing !!!
    double velLmt = 0.08; // 0.1 // unit in meters per sec
    double dura = sqrt(pow(r.in[0]-r.home[0],2)+pow(r.in[1]-r.home[1],2)+pow(r.in[2]-r.home[2],2))/velLmt*1000; // *1000 to change unit to ms
    double a[6], b[6], c[6]; // cubic coefficients
    double t = 0;
    cout << "Expected homing duration: " << dura <<"ms\n";
    if (dura == 0){ return; }

    for(int i = 0; i < 6; i++){
        // solve coefficients of equations for cubic
        a[i] = r.in[i];
        b[i] = 3 / (dura * dura) * (r.home[i] - r.in[i]);
        c[i] = -2 / (dura * dura * dura) * (r.home[i] - r.in[i]);
    }
    while (t <= dura){
        auto start = chrono::steady_clock::now();
        long dur = 0;
        
        // CUBIC equation
        for (int j = 0; j < 6; j++){
            r.in[j] = a[j] + b[j] * t * t + c[j] * t * t * t;
        }
        r.PrintIn();
        r.PoseToLength(r.in, r.out, railOffset);
        // cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << endl;
        
        SendMotorGrp(r);

        // Write to traking file
        ofstream myfile;
        myfile.open ("traking.txt");
        myfile << r.in[0] << " " << r.in[1] << " " << r.in[2] << " " << r.in[3] << " " << r.in[4] << " " << r.in[5] << endl;
        myfile << railOffset << endl;
        myfile.close();
        
        auto end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        cout << " Before sleep: " << dur << endl;
        
        double dif = MILLIS_TO_NEXT_FRAME - dur;
        if(dif > 0) { Sleep(dif); }
        // Sleep(MILLIS_TO_NEXT_FRAME);

        end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        cout << " Time elasped: " << dur << "\tTime left: " << dura - t << endl;
        t += MILLIS_TO_NEXT_FRAME;
        if(kbhit()){ // Emergency quit during trajectory control
            cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nr - Resume trajectory\n";
            char cmd;
            cin >> cmd;
            if(cmd == 'q' || cmd == 'Q'){
                cout << "Trajectory control aborted.\n";
                t = dura;
                break;
            }
        }
        if(limitType != 'C'){
            cout << "WARNING! Linear rail limits triggered. Please quit the programme and check the system.\n";
            return;
        }
    }
    cout << "Homing with trajectory completed\n";
}

bool ReadBricksFile(){ // Define which file to read here !!!
    ifstream file ("bricks.csv");
    vector<double> row;
    string line, word, temp;

    brickPos.clear();
    if(file.is_open()){
        while (getline(file, line)){
            row.clear();
            stringstream s(line);
            while (s >> word){
                row.push_back(stod(word)); // convert string to double stod()
            }
            for(int i=0; i<3; i++){ row[i] /= 1000; } // convert mm to m unit for xyz
            row[2] *= 1.01; // actual scale of bricks
            row[3] *= -1; // convert Adam's file from anticlockwise to clockwise in gripper
            //row[3] += 90; // convert Adam's file to robot rotation, 90deg offset
            if(row[3]>180){ row[3] -= 180; } // convert 360 degs to 180
            brickPos.push_back(row);
        }
        cout << "Completed reading brick position input file" << endl;
    }
    else{ cout << "Failed to read input file. Please check \"bricks.csv\" file." << endl; return false;
    }
    return true;
}

void HomeLinearRail(int n){
    double velLmt = -2000; // IMPORTANT!!!!! Set the linear homing speed here!!
    double hLmtOffset[4] = {-34000, -34000, -34000, -34000}; // Set offset from home switch to real "home" in motor counts units

    SysManager* myMgr = SysManager::Instance();
    INode &theNode = myMgr->Ports(2).Nodes(n);
    theNode.Motion.VelocityReachedTarget();
    theNode.Motion.MoveVelStart(velLmt);
            
    while(!kbhit()){
        switch(limitType){
        case 'A':
            cout << "InA detected!\n";
            theNode.Motion.MoveVelStart(0);
            while(!theNode.Motion.VelocityAtTarget()){} // wait till motor stopped
            theNode.Motion.AddToPosition(-theNode.Motion.PosnMeasured.Value()+hLmtOffset[n]); // Add offset here if applicable
            cout << "Reached homed swtich.\n";
            theNode.Motion.MovePosnStart(0, true);
            while(!theNode.Motion.MoveIsDone()){}
            cout << "Homing completed.\n";
            limitType = 'C';
            return;
        case 'B':
            cout << "InB detected!\n";
            velLmt *= -1;
            theNode.Motion.MoveVelStart(velLmt);
            if (velLmt > 0) { // ie moving upwards
                while(limitType != 'A') {} //wait till home switch is passed
                Sleep(2000);
                velLmt *= -1;
                theNode.Motion.MoveVelStart(velLmt);
                limitType = 'C'; // wait till the home is hit from upward side
                break;
            } // else, just break and wait till homed
            break;
        }
    }
    theNode.Motion.MoveVelStart(0);
    cout << "Linear rail motion interrupted\n";
}

void MN_DECL AttentionDetected(const mnAttnReqReg &detected){
    // Make a local, non-const copy for printing purposes
    mnAttnReqReg myAttns = detected;
    // Load the buffer with the string representation of the attention information
    myAttns.AttentionReg.StateStr(attnStringBuf, 512);
    // Print it out to the console
    printf("ATTENTION: port %d, node=%d, attn=%s\n", detected.MultiAddr >> 4, detected.MultiAddr, attnStringBuf);

    limitType = attnStringBuf[strlen(attnStringBuf)-2];
}