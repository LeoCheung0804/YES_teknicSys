#ifndef DeltaClient_H
#define DeltaClient_H

#include "vector"
#include "..\include\Logger.h"
#include "..\include\Robot.h"
#include "..\include\TrajectoryGenerator.h"
#include <iostream>
#include <string>
#include <fstream>
#include "..\tools\json.hpp"
#include <assert.h>
#pragma comment(lib, "User32.lib")

#pragma comment(lib, "ws2_32.lib")


using json = nlohmann::json;
using namespace std;
class DeltaClient{
private:
    SOCKET udpSocket;
    sockaddr_in serverAddr;
    int railNumber;
    bool isOnline;
    bool isMoveFinished;
    int MILLIS_TO_NEXT_FRAME = 50;
    string sendStr;
    bool *bArry;
    int workingMotor = 0;
    bool isConnected;
    bool useRail;
public:
    DeltaClient(bool isOnline=false, bool useRail=false);
    void Connect(int motorPortNumber, int railNumber);
    void Disconnect();
    bool IsConnected();
    void CalibrationMotor(int index, double currentCmdPos);
    void SelectWorkingMotor(int index);
    void MoveSelectedMotorCmd(double cmd, bool absolute=true);
    vector<int> GetMotorPosMeasured();
};
#endif