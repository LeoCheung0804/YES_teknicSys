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
#include "pose_to_length.h"
#include "Dependencies\sFoundation20\inc\pubSysCls.h"
#include "Dependencies\DynamixelSDK-3.7.31\include\dynamixel_sdk\dynamixel_sdk.h"

using namespace std;
using namespace sFnd;

bool SetSerialParams();
int CheckMotorNetwork();
int RunParaBlend(double point[7], bool showAttention = false);
void RunBricksTraj(bool groupSyncRead, int listOffset, bool showAttention = false, bool waitBtn = false);
void ReverseBricksTraj(bool groupSyncRead, int listOffset, bool showAttention = false);
void RunDemoTraj(bool groupSyncRead, bool showAttention = false);
void SendMotorGrp(bool IsTorque = false, bool IsLinearRail = false);
int32_t ToMotorCmd(int motorID, double length);
void TrjHome();
bool CheckLimits();
bool ReadBricksFile();
void HomeLinearRail(int n);
void MN_DECL AttentionDetected(const mnAttnReqReg &detected); // this is attention from teknic motors

vector<string> comHubPorts;
vector<INode*> nodeList; // create a list for each node
vector<vector<double>> brickPos;
vector<double> spcLimit;
unsigned int portCount;
char attnStringBuf[512]; // Create a buffer to hold the attentionReg information    
const int NodeNum = 8; // !!!!! IMPORTANT !!!!! Put in the number of motors before compiling the programme
const double RAIL_UP = 1.25, RAIL_DOWN = 0; // Linear rail upper and lower bound
const double endEffOffset = -0.125; // meters, offset from endeffector to ground
double step = 0.01; // in meters, for manual control
float targetTorque = -4.1; //2.5 in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
const float absTorqueLmt = 60.0; // Absolute torque limit, to be checked before commanding 8 motors to move simutaneously in length command
const int MILLIS_TO_NEXT_FRAME = 35, UserInput_Sec_Timeout = 15, SleepTime = 21; // note the basic calculation time is abt 16ms; sleep-time in 24 hr
double home[6] = {7.3, 6.2, -1.6, 0, 0, 0}; // home posisiton
double offset[8]; //12 L0, from "zero position", will be updated by "set home" command
double railOffset = 0.969599; // linear rails offset
double in1[6] = {1.5, 1.5, 1.4, 0, 0, 0};
double out1[8] = {2.87451, 2.59438, 2.70184, 2.40053, 2.46908, 2.15523, 2.65123, 2.35983}; //12 assume there are 8 motors + 4 linear rails
double a[6], b[6], c[6], d[6], e[6], f[6], g[6], tb[6]; // trajectory coefficients
char limitType = 'C'; // A for home, B for limits, C for default
char quitType = 'r'; // q for emergency quit, f for finish traj, e for error msg received, r for resume/default
int loopCount = 0, abbCount = 0; // log info for loop numbers and error occurs daily

/*dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0; // Dynamixel error
int32_t dxl1Pos = 0, dxl2Pos = 0, gpOpen = -30, gpClose = 80, neutralRot = 1820; // define some position reading, and gripper commands. neutralRot is 1024.90 deg // 1820 is 160 deg
int dxl_comm_result, rotationG, gripperG, dxl_pre_pos = 0;
const int DXL1_ID = 1, DXL2_ID = 2; //dxl 1 is rotation motor, dxl 2 is gripper motor
const int ADDR_TORQUE_ENABLE = 64, ADDR_GOAL_POSITION = 116, ADDR_PRESENT_POSITION = 132, ADDR_GOAL_CURRENT = 102, ADDR_PRESENT_CURRENT = 126; // Control table adresses
const int LEN_GOAL_POSITION = 4, LEN_PRESENT_POSITION = 4, LEN_GOAL_CURRENT = 2, LEN_PRESENT_CURRENT = 2, DXL_THRESHOLD = 10;*/
bool groupSyncRead = true;

HANDLE hComm; // Handle to the Serial port, https://github.com/xanthium-enterprises/Serial-Programming-Win32API-C
DWORD dwEventMask, BytesRead, dNoOfBytesWritten = 0;
bool Status;
char ComPortName[] = "\\\\.\\COM33"; // Name of the arduino Serial port(May Change) to be opened,
unsigned char tmp, msg[256], Ard_char[] = {'s'};

int main()
{     
    //// Open serial port for Arduino
    hComm = CreateFile(ComPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hComm == INVALID_HANDLE_VALUE){ cout << "Error: " << ComPortName << " cannot be opened.\n"; }
    else { cout << ComPortName << " opened.\n"; }
    if (!SetSerialParams()) { return -1; }
    
    //// initiallize cable robot motor network
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
    IPort &myPort = myMgr->Ports(2);
    pose_to_length(home, offset); // save offset values according to home pose

    cout << "Motor network available. Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\ns - Set current position as home\nh - Move to Home\n8 - Manually adjust cable lengths\nl - Linear rails motions\nu - Update current position from external file\nr - Reset Rotation to zero\ni - Info: show menu\nn - Move on to Next step" << endl;
    char cmd;
    try{
        do {
            bool allDone = false, stabilized = false;
            time_t now = time(0); time_t timeout = now + UserInput_Sec_Timeout; cmd = 'u'; // default value
            while(now < timeout){
                Sleep(50);
                now = time(0);
                if(kbhit()){ cin >> cmd; break; }
            }
            switch (cmd){
                case 'i':   // Show menu
                    cout << "Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\ns - Set current position as home\nh - Move to Home\n8 - Manually adjust cable lengths\nl - Linear rails motions\nu - Update current position from external file\nr - Reset Rotation to zero\ni - Info: show menu\nn - Move on to Next step\n";
                    break;
                case 'y':   // Loosen cables using positive torque
                    targetTorque = 1;
                case 't':   // Tighten cables according to torque **Only for 8 motors
                    cout << "Current target torque = " << targetTorque << endl;
                    for(INode* n : nodeList){ n->Motion.AccLimit = 200; }
                    while(!stabilized) {
                        SendMotorGrp(true);
                        Sleep(50);
                        for (int n = 0; n < NodeNum; n++){
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
                    targetTorque = -2.5;
                    break;
                case 's':   // Set zero
                    for (int n = 0; n < NodeNum; n++){
                        nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value()); // Zeroing the number space around the current Measured Position
                    }
                    copy(begin(home), end(home), begin(in1)); // copy home array into input array
                    cout << "Setting zero completed" << endl;
                    cout <<  "Home coordinates: " << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;

                    cout << "Do you want to set current rail position as zero? (k - OK)\n";;
                    cin >> cmd;
                    if(cmd == 'k'){
                        for (int n = NodeNum; n < NodeNum+4; n++){
                            nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value());
                        }
                        cout << "Linear rails are set to zero.\n";
                    }
                    else{ cout << "Current rail position may not be at zero\n"; }
                    break;
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
                    copy(begin(home), end(home), begin(in1)); // copy home array into input array
                    cout << "Homing completed" << endl;
                    break;
                case '8':   // Manual cable adjustment
                    cout << "0 to 7 - motor id to adjust cable length\n8 - move 4 linear rails together\na or d - increase or decrease cable length\nb - Back to previous menu\n";
                    while(cmd != 'b'){
                        cin >> cmd;
                        if('/' < cmd && cmd < NodeNum + 49){
                            int id = cmd - 48;
                            int sCount = ToMotorCmd(-1, step) / 5;
                            cout << "Motor "<< cmd <<" selected.\n";
                            do{
                                cmd = getch();
                                switch(cmd){
                                    case 'a':
                                        if(id == NodeNum){ for(int n = NodeNum; n<NodeNum+4; n++){ nodeList[n]->Motion.MovePosnStart(sCount); }}
                                        else { nodeList[id]->Motion.MovePosnStart(sCount); }
                                        break;
                                    case 'd':
                                        if(id == NodeNum){ for(int n = NodeNum; n<NodeNum+4; n++){ nodeList[n]->Motion.MovePosnStart(-sCount); }}
                                        else { nodeList[id]->Motion.MovePosnStart(-sCount); }
                                        break;
                                    case 'i':
                                        if(id == NodeNum){
                                            for(int n = NodeNum; n<NodeNum+4; n++){
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
                                        if(id == NodeNum){ cout << "Homing for linear rails are not implemented here.\n"; break; }
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
                        copy(in1, in1+3, begin(point)); // copy x,y,z position
                        cout << "Goal coordinates: " << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << ", " << point[4] << ", " << point[5] << ", " << point[6] << endl;
                        if(RunParaBlend(point) < 0) { cout << "Trajectory aborted.\n"; break; }
                        cout << "Rotation reset completed.\n";      
                    }
                    break;
                case 'l':   // Linear rail motions
                    cout << "0 to 3 - home linear rail individually\n4 - home all 4 linear rails automatically\nb - Back to previous menu\nany other keys - stop the linear rail from current motion\n";
                    while(cmd != 'b'){
                        cin >> cmd;
                        if('/' < cmd && cmd < 52){ // homing individually
                            int id = cmd - 48;
                            cout << "Homing linear rail #"<< cmd <<".\n";
                            HomeLinearRail(id);
                        }
                        else if(cmd == 52){ // homing 4 all tgt
                            for (int i = 0; i < 4 ; i++){
                                HomeLinearRail(i);
                            }
                            cout << "All linear rails are homed.\n";
                        }
                    }
                    cout << "Linear rail homing terminated\n";
                    break; 
                case 'u':   // Update in1[] and offset[] from csv file
                    ifstream file ("lastPos.txt"); //"lastPos.txt" or "currentPos.csv"
                    string temp;
                    int count = 0;
                    if(file.is_open()){
                        try{
                            while (file >> temp){
                                if(count > 5) { railOffset = stod(temp); break; } // reading the rail offset, then break while loop
                                in1[count++] = stod(temp); // convert string to double stod()
                            }
                            cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
                        }
                        catch(int e){ cout << "Check if currentPos.csv matches the in1 input no." << endl; }
                        // do{
                        //     cout << "Current linear rail offset: ";
                        //     cin >> railOffset; // do we need other constraits? ie 0 <= railOffset < 2
                        // }while(!cin.good() || railOffset>2);
                        
                        pose_to_length(in1, out1, railOffset);
                        for (int n = 0; n < nodeList.size(); n++){
                            int32_t step = ToMotorCmd(n, out1[n]);
                            nodeList[n]->Motion.PosnMeasured.Refresh();
                            nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value() + step);
                        }
                        cout << "Updating motor counts completed" << endl;
                        cout << "Current coordinates: " << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;
                        cout << "Motor internal counts: ";
                        for (int id = 0; id < nodeList.size(); id++){
                            nodeList[id]->Motion.PosnMeasured.Refresh();
                            cout << (double) nodeList[id]->Motion.PosnMeasured << "\t";
                        }
                        cout << endl;
                        cout << "Linear rail offset: " << railOffset << endl;
                        
                        now = time(0); timeout = now + UserInput_Sec_Timeout; cout << "!! If no input detected in " << UserInput_Sec_Timeout << " sec, programme will enter next section. !!" << endl;
                        cmd = 'n'; // default value to move on to next section
                        while(now < timeout){
                            Sleep(50);
                            now = time(0);
                            if(kbhit()){ cin >> cmd; break; }
                        }
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
    
    //// Initialize dynamexial gripper
    /*portHandler = dynamixel::PortHandler::getPortHandler("\\\\.\\COM31"); // Initialize PacketHandler and PacketHandler instance
    packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    // dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);       
    {    // Open dynamixel port
        if (portHandler->openPort()) { cout << "Succeeded to open Dynamixel port\n"; }
        else { cout << "Failed to open the port!\nPress any key to terminate...\n"; getch(); return -1; }
        // Set port baudrate
        if (portHandler->setBaudRate(57600)) { cout << "Succeeded to change Dynamixel baudrate!\n"; }
        else { cout << "Failed to change the baudrate!\nPress any key to terminate...\n"; getch(); return -1; }

        Sleep(20);
        // Enable Dynamixel#1 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        else { printf("Dynamixel#%d has been successfully connected \n", DXL1_ID); }

        Sleep(20);
        // Change Dynamixel#2 to current mode
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, 11, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        else { printf("Dynamixel#%d has been successfully connected \n", DXL2_ID); }

        Sleep(20);
        // Enable Dynamixel#2 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        else { printf("Dynamixel#%d has been successfully connected \n", DXL2_ID); }

        // // Setup Dynamixel#2 Current
        // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, 50, &dxl_error);
        // if (dxl_comm_result != COMM_SUCCESS) {cout << "Current Comm result: " << dxl_comm_result <<endl; }
        // else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        // else { printf("Goal current of Dynamixel#%d is set \n", DXL2_ID); }

        // Add parameter storage for present position value
        if(!groupSyncRead.addParam(DXL1_ID)){ cout << DXL1_ID << " groupSyncRead addparam failed\n"; }
        if(!groupSyncRead.addParam(DXL2_ID)){ cout << DXL2_ID << " groupSyncRead addparam failed\n"; }

        cout << endl;
    }*/
    // Set linear rail brakes as usually enabled
    myMgr->Ports(2).BrakeControl.BrakeSetting(0, BRAKE_PREVENT_MOTION);

    //// Read input .txt file
    cout << "Choose from menu for cable robot motion:\nt - Read from \"bricks.csv\" file for brick positions\nl - Loop through set num of bricks\nb - Loop and wait for Button input\nm - Manual input using w,a,s,d,r,f,v,g\nd - Run Demo loop\ni - Info: show menu\nn - Prepare to disable motors and exit programme" << endl;
    do {
        bool waitBtn = false;
        tm *fn; time_t now = time(0); time_t timeout = now + UserInput_Sec_Timeout; cmd = 'b'; // default value for button loop
        while(now < timeout){
            Sleep(50);
            now = time(0);
            if(kbhit()){ cin >> cmd; break; }
        }
        switch (cmd){
            case 'i':    // Show menu
                cout << "Choose from menu for cable robot motion:\nt - Read from \"bricks.csv\" file for brick positions\nl - Loop through set num of bricks\nb - Loop and wait for Button input\nm - Manual input using w,a,s,d,r,f,v,g\nd - Run Demo loop\ni - Info: show menu\nn - Prepare to disable motors and exit programme" << endl;
                break;
            case 't':   // Read brick file, plan trajectory
            case 'T':
                if(!ReadBricksFile()){ continue; } // Read "bricks.csv"
                RunBricksTraj(groupSyncRead, 0);
                break;
            case 'm':   // Manual wasdrf
            case 'M':
                cout << "Press 'q' to quit manual input anytime.\n'h' for Homing.\n'x' to adjust increment step size.\n";
                nodeList[8]->Port.BrakeControl.BrakeSetting(0, BRAKE_ALLOW_MOTION); // disable enable brake
                while(cmd != 'q' && cmd != 'Q'){
                    cmd = getch();
                    switch(cmd){
                        // case 'K': // Read Dynamixel#2 current position // TODO: cannot read feedback?
                        // case 'k':
                        //     if(packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_pre_pos, &dxl_error)){ cout << "Error in reading gripper positio\n"; }
                        //     else{ cout << "Gripper position: " << dxl_pre_pos << endl; }
                        //     continue;
                        /*case 'I':
                        case 'i':
                            if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; }
                            continue;
                        case 'O':
                        case 'o':
                            if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; }
                            continue;
                        case 'P':
                        case 'p':
                            cout << "Rotation: ";
                            cin >> rotationG;
                            if(cin.good()){
                                rotationG *= 11.26666;
                                if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in gripper rotation command\n"; };
                            }
                            continue; */
                        case 'W':
                        case 'w':
                            in1[1] += step;
                            break;
                        case 'S':
                        case 's':
                            in1[1] -= step;
                            break;
                        case 'A':
                        case 'a':
                            in1[0] -= step;
                            break;
                        case 'D':
                        case 'd':
                            in1[0] += step;
                            break;
                        case 'R':
                        case 'r':
                            in1[2] += step;
                            break;
                        case 'F':
                        case 'f':
                            in1[2] -= step;
                            break;
                        case 'G':
                        case 'g':
                            railOffset += step/20;
                            if(railOffset > RAIL_UP) { cout << "WARNING! Rail offset beyond upper bound!\n"; railOffset -= step/20; }
                            cout << "Current rail offset: " << railOffset << endl;
                            pose_to_length(in1, out1, railOffset);
                            SendMotorGrp(false, true);
                            continue;
                        case 'V':
                        case 'v':
                            railOffset -= step/20;
                            if(railOffset < RAIL_DOWN) { cout << "WARNING! Rail offset beyond lower bound!\n"; railOffset += step/20; }
                            cout << "Current rail offset: " << railOffset << endl;
                            pose_to_length(in1, out1, railOffset);
                            SendMotorGrp(false, true);
                            continue;
                        case 'H':
                        case 'h':
                            cout << "Homing...\n"; 
                            TrjHome();
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
                    cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
                    if(CheckLimits()){
                        pose_to_length(in1, out1, railOffset);
                        cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << " " <<  out1[4] << " " << out1[5] << " " << out1[6] << " " << out1[7] << " " <<  out1[8] << " " << out1[9] << " " << out1[10] << " " << out1[11] << endl;
                        SendMotorGrp();
                        
                        Sleep(step*1000);
                    }
                    else{
                        cout << "WARNING: Intended position out of bound!\n";
                        switch(cmd){
                            case 'W':
                            case 'w':
                                in1[1] -= step;
                                break;
                            case 'S':
                            case 's':
                                in1[1] += step;
                                break;
                            case 'A':
                            case 'a':
                                in1[0] += step;
                                break;
                            case 'D':
                            case 'd':
                                in1[0] -= step;
                                break;
                            case 'R':
                            case 'r':
                                in1[2] -= step;
                                break;
                            case 'F':
                            case 'f':
                                in1[2] += step;
                                break;
                        }
                    }
                }
                nodeList[8]->Port.BrakeControl.BrakeSetting(0, BRAKE_PREVENT_MOTION); // enable brake afterwards
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
                    ReverseBricksTraj(groupSyncRead, listOffset, true);
                    if(quitType == 'q' || quitType == 'Q' || quitType == 'e'){ break; }
                    RunBricksTraj(groupSyncRead, listOffset, true, waitBtn);
                    if(quitType == 'q' || quitType == 'Q' || quitType == 'e'){ break; }
                    loopCount += 1;
                    now = time(0); fn = localtime(&now); if(fn->tm_hour >= SleepTime) { break; } // quit loop after sleep time
                    if(quitType != 'f' && !waitBtn){
                        cout << "Taking a 3 minute rest~ ^O^\n\n";
                        Sleep(1000*180);
                    }
                }
                cout << "Quit looping trajectory.\n";}
                break;
            case 'd':   // Temporary Demo function for moving top bricks around
            case 'D':
                ifstream file ("demo.csv");
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
                        if(row[3]>180){ row[3] -= 180; } // convert 360 degs to 180
                        brickPos.push_back(row);
                    }
                    cout << "Completed reading brick position input file" << endl;
                }
                else{ cout << "Failed to read input file. Please check \"bricks.csv\" file." << endl; break;
                }

                loopCount = 0;

                quitType = 'r';
                while(quitType != 'f' && quitType != 'F'){ // if not running the final loop.....
                    // always reverse from a complete built, then rebuild it
                    RunDemoTraj(groupSyncRead, true);
                    loopCount += 1;
                    if(quitType == 'q' || quitType == 'Q' || quitType == 'e'){ break; }
                    cout << "Taking a 5 sec rest ;) \n"; Sleep(5000); // Sleep for a while before next run for every 8 loop 
                    if(!(loopCount%25)){ cout << "Taking a 15 minute rest~ ^O^ \n\n"; Sleep(1000*900); } // Sleep for a while before next run for every 400 loop
                }
                cout << "Quit looping demo trajectory.\n";
                break;
        }
        now = time(0); fn = localtime(&now);
        if(fn->tm_hour >= SleepTime) { cout << "\nATTENTION: Exhibit closing time. Thank you!\n" << endl; cmd = 'n'; }
    } while(cmd != 'n' && quitType != 'e');

    //// Safe system shut down, safe last pos and emegency shut down
    // Saving last position before quiting programme
    cout << "Saving last position...\n";
    ofstream myfile;
    myfile.open ("lastPos.txt");
    myfile << in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
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
    
    {   // Send 'p'signal to arduino for shutting down
        // Ard_char[0] = 'p';
        // if (!(bool)WriteFile(hComm, Ard_char, 1, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; }
        CloseHandle(hComm); //Close the Serial Port
    }

    /*{   // Disable Dynamixel Torque
        packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, 0, &dxl_error);
        Sleep(20);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        Sleep(20);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }

        // Close port
        portHandler->closePort();
    }
    cout << "Dynamixel is closed correctly\n"; */

    for(int i = 0; i < nodeList.size(); i++){ //Disable Nodes
        nodeList[i]->EnableReq(false);
    }
    nodeList.clear();
    myMgr->PortsClose(); // Close down the ports
    cout << "Teknic motors are disabled\n";
    return 1;
}

bool SetSerialParams(){
    // Set parameters for serial port
    DCB dcbSerialParams = { 0 }; // Initializing DCB structure
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    Status = GetCommState(hComm, &dcbSerialParams); // retreives  the current settings
    if (Status == false){ cout << "Error in GetCommState()\n"; return false; }

    dcbSerialParams.BaudRate = CBR_115200;// Setting BaudRate
    dcbSerialParams.ByteSize = 1;         // Setting ByteSize = 8
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
    timeouts.ReadTotalTimeoutConstant    = 50; //50
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(hComm, &timeouts) == FALSE){ cout << "Error! in Setting Time Outs\n"; return false; }
    // Set recieve mask                
    if (!(bool)SetCommMask(hComm, EV_RXCHAR)){ cout << "Error! in Setting CommMask\n"; }
    
    return true;
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
        myPort.BrakeControl.BrakeSetting(0, BRAKE_AUTOCONTROL); // do we need this?
        myPort.BrakeControl.BrakeSetting(1, BRAKE_AUTOCONTROL);
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

            if (i == 2) { // Set the Node's Attention Mask to generate attentions on "Enabled" on every linear rail nodes
                attnReg attnReq;
                attnReq.cpm.InA = 1;
                attnReq.cpm.InB = 1;
                theNode.Adv.Attn.Mask = attnReq;

                theNode.Adv.Attn.ClearAttn(attnReq);
                theNode.EnableReq(true);
            }
        }
        if (i == 2){ // For linear rail port
            myPort.Adv.Attn.Enable(true); // Enable the attentions for this port, ie Port#2 for linear rails
            myPort.Adv.Attn.AttnHandler(AttentionDetected); // Register handler at the port level, just for illustrative purposes.
        }
    }
    return 0;
}

int RaiseRailTo(double target){ // !!! Define velocity limit !!!
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
        pose_to_length(in1, out1, railOffset);
        cout << "OUT: "<<  out1[4] << "\t" << out1[5] << "\t" << out1[6] << "\t" << out1[7] << "\t" <<  out1[8] << "\t" << out1[9] << "\t" << out1[10] << "\t" << out1[11] << endl;

        SendMotorGrp(false, true);

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

int RunParaBlend(double point[7], bool showAttention){
    float vMax[6] = {.4, .4, .4, 0.8, 0.8, 0.8}; // m/s, define the maximum velocity for each DoF
    float aMax[6] = {80, 80, 80, 10, 10, 10}; // m/s^2, define the maximum acceleration for each DoF
    double sQ[6], Q[6], o[6];
    double dura = point[6];
    double unitV = sqrt(pow(point[0]-in1[0],2)+pow(point[1]-in1[1],2)+pow(point[2]-in1[2],2)); // the root to divide by to get unit vector
    cout << "Will run for " << dura << "ms...\n";
    if(dura <= 200){ return 0; } // Don't run traj for incorrect timing 

    // Solve parabolic blend coefficients for each DoF
    for(int i = 0; i < 6; i++){
        sQ[i] = in1[i]; // start point, from current position
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
                in1[j] = a[j] + b[j] * t * t;
            }
            else if(t <= point[6]-tb[j]){
                in1[j] = c[j] + d[j] * t;
            }
            else{
                in1[j] = e[j] + f[j] * t + g[j] * t * t;
            }
        }
        // get absolute cable lengths in meters
        // cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        pose_to_length(in1, out1, railOffset);
        // cout << "OUT: "<<  out1[0] << "\t" << out1[1] << "\t" << out1[2] << "\t" << out1[3] << "\t" <<  out1[4] << "\t" << out1[5] << "\t" << out1[6] << "\t" << out1[7] << endl;

        SendMotorGrp();
        if(quitType == 'e'){ // Motor error message?
            cout << "WARNING! Motor error message received. System will now shut now.\n";
            return -4;
        }

        // Write to traking file
        ofstream myfile;
        myfile.open ("traking.txt");
        myfile << in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
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

double ScaleRailLvl(double brickLvl){
    double output = brickLvl * 0.8;
    if (output > 1.2){ return 1.2; } // Upper limit of linear rail
    else if (output < 0) { return 0; } // lower limit of linear rail
    return output;
}

//void RunBricksTraj(const dynamixel::GroupSyncRead &groupSyncRead, int listOffset, bool showAttention, bool waitBtn){
void RunBricksTraj(bool groupSyncRead, int listOffset, bool showAttention, bool waitBtn){
    // 0.75 0.865 1.6 p10 //{0.6723, 1.3195, 1.4611
    double brickPickUp[7] = {0.765, 0.8693, 1.604, 0, 0, 0, 10}; // !!!! Define the brick pick up point !!!!, the last digit is a dummy number for time duration.
    double safePt[3] = {1.5, 1.37, 1.65}; // a safe area near to the arm // 0.21 safe height from ABB
    double goalPos[7] = {2, 2, 1, 0, 0, 0, 10}; // updated according to brick position
    double velLmt = 0.14; // meters per second
    double safeT = 1200; // in ms, time to raise to safety height
    double safeH = 0.08; // meter, safety height from building brick level
    double currentBrkLvl = railOffset; // meter, check if the rail offset is the same as target BrkLvl
    double dura = 0;

    // Go through the given bricks
    for (int i = 0; i < brickPos.size(); i++) {
        ofstream myfile; int timeout_i = 0;

        // Check if rails need to be raised
        if(ScaleRailLvl(brickPos[i][2] - 0.04) != currentBrkLvl){
            currentBrkLvl = ScaleRailLvl(brickPos[i][2] - 0.0404); // Offset one brick height from building levei, ie 0.0404m
            if(RaiseRailTo(currentBrkLvl) < 0) { cout << "Trajectory aborted.\n"; return; } // raise rail to the building brick level
        }

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
        Ard_char[0] = 'g'; // Signal arduino to release ABB gripper and hard code waiting
        if (!(bool)WriteFile(hComm, Ard_char, 1, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; quitType = 'q'; return; }
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, 114, &dxl_error)){ cout << "Error in rotating gripper\n"; return; } //////////////////114 is 10 deg
        Sleep(10); // short break between RS-485 communication
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        goalPos[6] = sqrt(pow(brickPickUp[0]-in1[0],2)+pow(brickPickUp[1]-in1[1],2)+pow(brickPickUp[2]+0.21-in1[2],2))/velLmt*1000; // calculate time // 0.21 is height above brick pickup
        copy(brickPickUp, brickPickUp+3, begin(goalPos));
        goalPos[2] += 0.21; // 0.21 safe height from ABB
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // Go to safe height above pick up
        Sleep(100);
        // Wait for brick to be ready from ABB
        msg[0] = 'w';
        while(msg[0]!='d'){
            // if(kbhit()){ // manual input
            //     cout << "\nmanual input\n";
            //     cin >> tmp;
            //     if(tmp == 'd'){msg[0]='d'; cout << "Brick is ready from ABB :) \n"; break;}
            // }
            Status = WaitCommEvent(hComm, &dwEventMask, NULL); // wait till brick is ready from ABB
            if (Status == false){ cout << "Error in setting WaitCommEvent()\n";} //quitType = 'q'; return; }
            else {
                ReadFile(hComm, &tmp, sizeof(tmp), &BytesRead, NULL);
                msg[0] = tmp;
                cout << msg[0] << ";";
                if(msg[0]=='d'){ cout << "Brick is ready from ABB :) \n"; }
            }
        }
        
        // fall and pick up brick
        brickPickUp[6] = safeT*1.2;
        if(RunParaBlend(brickPickUp) < 0) { cout << "Trajectory aborted.\n"; return; } // Go pick up
        Sleep(600); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; return; }
        Sleep(800); // wait for grippper to close
        Ard_char[0] = 'r'; // Signal arduino to release ABB gripper and hard code waiting
        if (!(bool)WriteFile(hComm, Ard_char, 1, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; quitType = 'q'; return; }
        
        // Wait till brick is released from ABB
        int msgCnt = 0; Ard_char[0] = 'o'; // Hard code signal arduino to open ABB gripper
        Status = WaitCommEvent(hComm, &dwEventMask, NULL); // wait till brick is ready from ABB
        if (Status == false){ cout << "Error in setting WaitCommEvent()\n";} //quitType = 'q'; return; }
        else {
            do{
                ReadFile(hComm, &tmp, sizeof(tmp), &BytesRead, NULL);
                msg[msgCnt] = tmp;
                if(tmp == 'o'){ cout << " -o- Received the gripper opening done signal :) \n"; }
                if (msgCnt<250){ msgCnt++; }
                else{
                    msgCnt = 0; // Restart msg count
                    timeout_i += 1;
                    if (!(timeout_i%3)){ // Send hard code open to ABB after some timeout; 20 loops is ard 5 min
                        Ard_char[0] = 'c'; // Hard code signal arduino to close ABB gripper
                        if (!(bool)WriteFile(hComm, Ard_char, 1, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; quitType = 'q'; return; }
                        Sleep(3000); // Wait a bit before sending next signal
                        Ard_char[0] = 'o'; // Hard code signal arduino to close ABB gripper
                        if (!(bool)WriteFile(hComm, Ard_char, 1, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; quitType = 'q'; return; }
                        cout << "ATTENTION: hard code closing is sent to ABB\n";
                        abbCount += 1;
                    }
                }
                if(kbhit()){ // Emergency quit during trajectory control
                    cout << "\nSystem interruption (Waiting for ABB)!! Do you want to quit the trajectory control?\nq - Quit trajectory\nf - Finish this loop of motion\nr - Resume trajectory and Request ABB gripper release\n";
                    cin >> quitType;
                    if(quitType == 'q' || quitType == 'Q'){
                        cout << "Trajectory emergency quit\n";
                        return;
                    }
                    if(quitType == 'r' || quitType == 'R'){
                        Ard_char[0] = 'c'; // Hard code signal arduino to close ABB gripper
                        if (!(bool)WriteFile(hComm, Ard_char, 1, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; quitType = 'q'; return; }
                        Sleep(3000); // Wait a bit before sending next signal
                        Ard_char[0] = 'o'; // Hard code signal arduino to close ABB gripper
                        if (!(bool)WriteFile(hComm, Ard_char, 1, &dNoOfBytesWritten, NULL)){ cout << "Arduino writing error: " << GetLastError() << endl; quitType = 'q'; return; }
                        cout << "ATTENTION: hard code closing is sent to ABB\n";
                        abbCount += 1;
                    }
                }
            } while (tmp != 'o'); // while (BytesRead > 0 && tmp != 'o');
        }
        Sleep(4000); // wait for ABB to return stand by pos??? // fastest 7000

        // Go to building level
        if(showAttention) { cout << "Raising brick from ABB\n"; }
        copy(brickPickUp, brickPickUp+3, begin(goalPos));
        goalPos[2] += 0.14;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise the brick from robot arm
        Sleep(50); // Wait for ABB to leave
        if(showAttention) { cout << "Going to building level\n"; }
        // rotationG = brickPos[i][3] * 11.26666;; // conversion from angle to motor command
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        copy(safePt, safePt+3, begin(goalPos)); // safe point
        goalPos[6] = sqrt(pow(safePt[0]-in1[0],2)+pow(safePt[1]-in1[1],2)+pow(safePt[2]-in1[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point
        goalPos[2] = brickPos[i][2] + endEffOffset + safeH; // brick level with safe height
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise the brick to building level

        // Go to brick placing position
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];
        goalPos[1] = brickPos[i][1];
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }

        // Place brick
        if(showAttention) { cout << "Placing brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        Sleep(200); //Wait a while after placing brick
        
        // Rise and leave building area, stand by for next brick pick up
        if(showAttention) { cout << "Going to stand by position\n"; }
        goalPos[2] += safeH;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, neutralRot, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y,z position
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << railOffset << endl;
        cout << "----------Completed brick #" << i + 1 + listOffset <<"----------" << endl;
    }
    // packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, 0, &dxl_error);
    copy(home, home+3, begin(goalPos));
    goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
    if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // home after building all
    cout << "Photo time~~\n"; Sleep(3000);
}

// void ReverseBricksTraj(const dynamixel::GroupSyncRead &groupSyncRead, int listOffset, bool showAttention){
void ReverseBricksTraj(bool groupSyncRead, int listOffset, bool showAttention){
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
        // Check if rails need to be raised
        if(ScaleRailLvl(brickPos[i][2] - 0.04) != currentBrkLvl){
            currentBrkLvl = ScaleRailLvl(brickPos[i][2] - 0.04); // Offset one brick height from building levei, ie 0.04m
            if(RaiseRailTo(currentBrkLvl) < 0) { cout << "Trajectory aborted.\n"; return; } // raise rail to the building brick level
        }

        // Update index file for grasshopper display
        // ofstream myfile;
        // myfile.open ("indexBrk.txt");
        // myfile << i + 1 + listOffset;
        // myfile.close();

        // Go to building level
        if(showAttention) { cout << "Going to building level\n"; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        copy(in1, in1+2, begin(goalPos));
        goalPos[2] = brickPos[i][2] + endEffOffset + safeH; // brick level
        goalPos[6] = sqrt(pow(goalPos[2]-in1[2],2))/velLmt*1000 + 800; // calculate time + hardcode time to smooth out sudden jump
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise from current pos to builing level
        
        // Go to brick placing position
        // rotationG = brickPos[i][3] * 11.26666;; // conversion from angle to motor command
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];                       
        goalPos[1] = brickPos[i][1];
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        
        // Retrieve brick
        if(showAttention) { cout << "Retrieving brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; return; }
        Sleep(1500); // Wait for gripper to close

        // Rise and leave building area, stand by for next brick pick up
        if(showAttention) { cout << "Going to drop off position\n"; }
        goalPos[2] += safeH;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, neutralRot, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y position
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        // Dropping off a brick
        if(showAttention) { cout << "Dropping off the brick\n"; }
        brickDropOff[6] = sqrt(pow(brickDropOff[0]-in1[0],2)+pow(brickDropOff[1]-in1[1],2)+pow(brickDropOff[2]-in1[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(brickDropOff) < 0) { cout << "Trajectory aborted.\n"; return; }
        brickDropOff[6] = 800; // Safe time for dropping
        brickDropOff[2] -= 0.02; // go down a little more before real drop
        if(RunParaBlend(brickDropOff) < 0) { cout << "Trajectory aborted.\n"; return; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        Sleep(500); // wait a little after dropping brick
        brickDropOff[2] += 0.02; // go up a little more after real drop
        if(RunParaBlend(brickDropOff) < 0) { cout << "Trajectory aborted.\n"; return; }
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe point
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        cout << "----------Retrieved brick #" << i + 1 + listOffset <<"----------" << endl;
    }
    // packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, 0, &dxl_error);
    copy(home, home+3, begin(goalPos));
    goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
    if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // home after retrieving all
}

// void RunDemoTraj(const dynamixel::GroupSyncRead &groupSyncRead, bool showAttention){ 
void RunDemoTraj(bool groupSyncRead, bool showAttention){ 
    double goalPos[7] = {2, 2, 1, 0, 0, 0, 10}; // updated according to brick position
    const double velLmt = 0.12; // meters per second
    const double safeT = 1500; // in ms, time to raise to safety height
    const double safeH = 0.08; // meter, safety height from building brick level
    double currentBrkLvl = railOffset; // meter, check if the rail offset is the same as target BrkLvl
    double dura = 0;
    
    if (brickPos.size()%2) { cout << "demo.csv line no. inconsistant, please check file\n"; return; } // Check if the brick pos file is even number

    // Go through the given bricks
    for (int i = 0; i < brickPos.size(); i++) {
        // Check if rails need to be raised
        if(ScaleRailLvl(brickPos[i][2] - 0.04) != currentBrkLvl){
            currentBrkLvl = ScaleRailLvl(brickPos[i][2] - 0.04); // Offset one brick height from building levei, ie 0.04m
            if(RaiseRailTo(currentBrkLvl) < 0) { cout << "Trajectory aborted.\n"; return; } // raise rail to the building brick level
        }

        // Go to building level
        if(showAttention) { cout << "Going to building level\n"; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        copy(in1, in1+2, begin(goalPos));
        goalPos[2] = brickPos[i][2] + endEffOffset + safeH; // brick level, WITH  end effector offset?????
        goalPos[6] = sqrt(pow(goalPos[2]-in1[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise from current pos to builing level
        
        // Go to brick placing position
        Sleep(500);
        // rotationG = brickPos[i][3] * 11.26666; // conversion from angle to motor command
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];                       
        goalPos[1] = brickPos[i][1];
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        
        // Retrieve brick
        if(showAttention) { cout << "Retrieving brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; return; }
        Sleep(1500); // Wait for gripper to close

        // Rise and leave building area, stand by for next brick pick up
        if(showAttention) { cout << "Going to drop off position\n"; }
        goalPos[2] += safeH;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
        

        ///////////////////////////////////// Place brick in second location /////////////////////////////////////
        i += 1; // Add to next brick position

        // Rotate brick to next place down brick posisiton angle
        Sleep(500);
        // rotationG = brickPos[i][3] * 11.26666; // conversion from angle to motor command
        // if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        
        // Go to brick placing position
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];                       
        goalPos[1] = brickPos[i][1];
        goalPos[2] = brickPos[i][2] + endEffOffset + safeH; // With end effector offset
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }

        // Place brick
        if(showAttention) { cout << "Placing brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        // if(packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        Sleep(500); //Wait a while after placing brick
        goalPos[2] += safeH;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
              
        cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << railOffset << endl;
        cout << "----------Completed brick #" << i + 1 <<"----------" << endl;
    }
    copy(home, home+3, begin(goalPos));
    goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
    if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // home after each loop
}

int32_t ToMotorCmd(int motorID, double length){ // applicable for all 12 motors
    double scale = 490310.0949; //509295.818; // 6400 encoder count per revoltion, 25 times gearbox, 50mm spool radias. ie 6400*25/(2*pi*0.05) 
    if(motorID >= NodeNum) {
        scale = 38400000; // 38400000; // 6400 encoder count per revoltion, 30 times gearbox, linear rail pitch 5mm. ie 6400*30/0.005 
        return length * scale;
    }
    else if(motorID == -1) { return length * scale; }
    return (length - offset[motorID]) * scale;
}

void SendMotorCmd(int n){
    ofstream myfile;
    // convert to absolute cable length command
    try{
        int32_t step = ToMotorCmd(n, out1[n]);
        nodeList[n]->Motion.MoveWentDone();
        nodeList[n]->Motion.MovePosnStart(step, true, true); // absolute position
        float trq = nodeList[n]->Motion.TrqMeasured.Value();
        if(abs(trq)>absTorqueLmt){
            myfile.open("log.txt", ios::app);
            myfile << "Motor [" << n << "] exceeds torque limit: " << trq;
            myfile.close();
            quitType = 'e';
            cout << "ATTENTION: Motor [" << n << "] exceeds torque limit: " << trq << endl;
        }
        nodeList[n]->Motion.Adv.TriggerGroup(1);

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

void SendMotorTrq(int n){
    nodeList[n]->Motion.TrqCommanded.Refresh();
    float currentTorque = nodeList[n]->Motion.TrqCommanded.Value();

    if(currentTorque > targetTorque){ nodeList[n]->Motion.MoveVelStart(-300); }
    else if (currentTorque < targetTorque - 1.8){ nodeList[n]->Motion.MoveVelStart(150); cout << "Too much torque!!\n";}
    else{ nodeList[n]->Motion.MoveVelStart(-10);}
    printf("Node[%d], current torque: %f\n", n, currentTorque);
}

void SendMotorGrp(bool IsTorque, bool IsLinearRail){
    SysManager* myMgr = SysManager::Instance();
    IPort &myPort = myMgr->Ports(0);
    void (*func)(int){ SendMotorCmd };
    if(IsTorque){ func = SendMotorTrq; }
    int n = IsLinearRail? 4 : 0; // offset in nodeList
    
    thread nodeThreads[NodeNum];
    for(int i = 0; i < NodeNum; i++){
        nodeThreads[i] = thread((*func), i + n); 
    }
    for(int i = 0; i < NodeNum; i++){
        nodeThreads[i].join();
    }
    // myPort.Adv.TriggerMovesInGroup(1);
    if (quitType!='e'){ myPort.Adv.TriggerMovesInGroup(1); } // Only move all if no error is caugth
}

void TrjHome(){// !!! Define the task space velocity limit for homing !!!
    double velLmt = 0.1; // unit in meters per sec
    double dura = sqrt(pow(in1[0]-home[0],2)+pow(in1[1]-home[1],2)+pow(in1[2]-home[2],2))/velLmt*1000; // *1000 to change unit to ms
    double t = 0;
    cout << "Expected homing duration: " << dura <<"ms\n";
    if (dura == 0){ return; }

    for(int i = 0; i < 6; i++){
        // solve coefficients of equations for cubic
        a[i] = in1[i];
        b[i] = 3 / (dura * dura) * (home[i] - in1[i]);
        c[i] = -2 / (dura * dura * dura) * (home[i] - in1[i]);
    }
    while (t <= dura){
        auto start = chrono::steady_clock::now();
        long dur = 0;
        
        // CUBIC equation
        for (int j = 0; j < 6; j++){
            in1[j] = a[j] + b[j] * t * t + c[j] * t * t * t;
        }
        cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        pose_to_length(in1, out1, railOffset);
        // cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << endl;
        
        SendMotorGrp();

        // Write to traking file
        ofstream myfile;
        myfile.open ("traking.txt");
        myfile << in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
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

bool CheckLimits(){
    if(spcLimit.empty()){
        ifstream file ("limit.csv"); // Read limit file
        string temp;
        if(file.is_open()){
            while (file >> temp){
                spcLimit.push_back(stod(temp)); // convert string to double stod()
            }
            cout << "Completed reading external limit file" << endl;
        }
    }
    for (int i = 0; i < 3; i++){
        if(spcLimit[i*2]>in1[i] || in1[i]>spcLimit[i*2+1]){ return false; }
    }
    return true;
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

void MN_DECL AttentionDetected(const mnAttnReqReg &detected)
{
    // Make a local, non-const copy for printing purposes
    mnAttnReqReg myAttns = detected;
    // Load the buffer with the string representation of the attention information
    myAttns.AttentionReg.StateStr(attnStringBuf, 512);
    // Print it out to the console
    printf("ATTENTION: port %d, node=%d, attn=%s\n", detected.MultiAddr >> 4, detected.MultiAddr, attnStringBuf);

    limitType = attnStringBuf[strlen(attnStringBuf)-2];
}