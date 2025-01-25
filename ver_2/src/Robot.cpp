#include "..\include\Robot.h"
#include "..\tools\json.hpp"
#include "..\Dependencies\eigen-3.3.7\Eigen\Dense"
#include "..\include\TrajectoryGenerator.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <conio.h>

using json = nlohmann::json;

Robot::Robot() {};

// Constructor, based on the requested model
Robot::Robot(string robotConfigPath)
{
    this->UpdateModelFromFile(robotConfigPath);
}

void Robot::Connect()
{

    if (this->isOnline)
    {
        if (!this->posLogger.OpenFile("log\\pos.log"))
        {
            cout << "Cannot open pos logger file. System will now exit." << endl;
            exit(-1);
        }
    }

    this->isConnected = true;
    // Init BLE Nodes
    if (this->useGripper)
    {
        this->gripper = GripperController(this->isOnline, this->useGripper);
        this->gripper.Connect(this->gripperCommPort);
        if (!this->gripper.IsConnected())
        {
            cout << "Error: Gripper not connected." << endl;
            this->isConnected = false;
        }
    }

    // Init Brake Motor Nodes
    if (this->useRailBraker || this->useCableBraker)
    {
        this->brake = BrakeController(this->isOnline);
        this->brake.UseCableBrake(this->cableMotorBrakeNum);
        this->brake.UseRailBrake(this->railMotorNum);
        this->brake.Connect(this->railBrakeCommPort);
        if (!this->brake.IsConnected())
        {
            cout << "Error: Brake not connected." << endl;
            this->isConnected = false;
        }
    }

    // Init Rail Motor Nodes
    if (this->useRailMotor)
    {
        this->rail = RailController(this->isOnline, this->useRailMotor);
        this->rail.Connect(851, this->railMotorNum);
        if (!this->rail.IsConnected())
        {
            cout << "Error: Rail motors not connected." << endl;
            this->isConnected = false;
        }
        vector<int> railMotorCmd = this->rail.GetMotorPosMeasured();
        for(int i = 0; i < this->railMotorNum; i++){
            int currentPos = railMotorCmd[i]; // in internal counts
            cout << "Rail " << i << " current pos: " << currentPos << endl;
            // this->rail.MoveMotorCmdAbsuloteById(i, currentPos);
            this->brake.OpenRailBrakeByIndex(i);
            int targetPos = this->RailMotorLengthToCmd(i, this->railOffset[i]); // length offset to internal counts
            cout << "Rail " << i << " target pos: " << targetPos << endl;
            this->rail.MoveMotorCmdAbsuloteById(i, targetPos);
            // this->brake.CloseRailBrakeByIndex(i);
        }

    }

    // Init Cable Motor Nodes
    if (this->useCableMotor)
    {
        this->cable = CableController(this->isOnline, this->useCableMotor);
        this->brake.OpenAllCableBrake();
        this->cable.Connect(this->cableMotorNum);
        this->brake.CloseAllCableBrake();
        if (!this->cable.IsConnected())
        {
            cout << "Error: Cable motors not connected." << endl;
            this->isConnected = false;
        }
    }
}

void Robot::Disconnect()
{
    if (this->gripper.IsConnected())
        this->gripper.Disconnect();
    if (this->brake.IsConnected()){
        this->brake.CloseAllRailBrake();
        this->brake.CloseAllCableBrake();
    if (this->cable.IsConnected())
        this->cable.Disconnect();
    if (this->rail.IsConnected())
        this->rail.Disconnect();
    }
    this->isConnected = false;

    if (this->isOnline)
        this->posLogger.CloseFile();
}

bool Robot::IsConnected() { return this->isConnected; }

bool Robot::IsValid() { return this->isValidModel; }

bool Robot::CheckLimits()
{
    for (int i = 0; i < 3; i++)
    {
        if (this->endEffectorPos[i] < this->endEffectorPosLimit[i * 2])
            return false;
        if (this->endEffectorPos[i] > this->endEffectorPosLimit[i * 2 + 1])
            return false;
    }
    return true;
}

void Robot::PrintEEPos()
{
    cout << "Current EE Pos: " << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->endEffectorPos[i] << "                   " << endl;
    }
}

void Robot::PrintBrickPickUpPos()
{
    cout << "Brick Pick Up Pos: ";
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->brickPickUpPos[0] << "    " << endl;
    }
}

void Robot::PrintRailOffset()
{ // Print railOffset[]
    cout << "Rail Offset: " << endl;
    for (int i = 0; i < this->railMotorNum; i++)
    {
        cout << "\t" << "Cable " << i << ": " << this->railOffset[i] << "    " << endl;
    }
}

void Robot::PrintCableLength()
{
    cout << "Cable Length: ";
    vector<double> cableLength = this->EEPoseToCableLength(this->endEffectorPos);
    for (int i = 0; i < this->cableMotorNum; i++)
    {
        cout << cableLength[i] << " ";
    }
    cout << endl;
    cout << "Rail Offset: ";
    for (int i = 0; i < this->railMotorNum; i++)
    {
        cout << this->railOffset[i] << " ";
    }
    cout << endl;
}

void Robot::PrintHomePos()
{ // Print home[]
    cout << "HOME Pos: ";
    for (int i = 0; i < 6; i++)
    {
        cout << homePos[i] << " ";
    }
    cout << endl;
}

void Robot::UpdateModelFromFile(string filename, bool reconnect)
{ // Read model.json file
    if (reconnect)
        this->Disconnect();
    json model;
    try
    {
        ifstream file(filename, ifstream::binary);
        if (!file.good())
        {
            cout << "Error: File Not Exist. Keep using current config." << endl;
            return;
        }
        file >> model;
        cout << "Importing " << filename << ": " << model.value("ModelName", "NOT FOUND") << endl;

        this->isOnline = model.value("isOnline", false);

        // Initialize parameters
        this->cableMotorNum = model.value("cableMotorNum", 8); // Default value set in 2nd input
        this->railMotorNum = model.value("railMotorNum", 4);
        this->railMotorCableMotorOffset = model.value("railMotorCableMotorOffset", 2);
        this->absTrqLmt = model.value("absTrqLmt", 10);
        this->endEffToGroundOffset = model.value("endEffToGroundOffset", -0.28);
        this->targetTrq = model.value("targetTorque", -2.5);
        this->cableMotorScale = model.value("cableMotorScale", 509295);
        this->railMotorScale = model.value("railMotorScale", 38400000);
        this->pulleyRadius = model.value("pulleyRadius", 0.045);
        this->rotationalAngleOffset = model.value("rotationalAngleOffset", 21.8);
        this->rotationalDistanceOffset = model.value("rotationalDistanceOffset", -0.0075);
        this->gripperCommPort = model.value("gripperCommPort", "\\\\.\\COM26");
        this->railBrakeCommPort = model.value("railBrakeCommPort", "\\\\.\\COM26");
        this->brickPickUpOffset = model.value("brickPickUpOffset", 0.12);
        this->endEffVelLmt = model.value("endEffVelLmt", 0.45);
        this->railVelLmt = model.value("railVelLmt", 0.01);
        this->useIndividualCableScale = model.value("useIndividualCableScale", false);
        this->useGripper = model.value("useGripper", false);
        this->useCableMotor = model.value("useCableMotor", false);
        this->useCableBraker = model.value("useCableBraker", false);
        this->useRailMotor = model.value("useRailMotor", false);
        this->useRailBraker = model.value("useRailBraker", false);
        this->cable.SetTrqLmt(this->absTrqLmt);
        this->MILLIS_TO_NEXT_FRAME = model.value("controlFreq", 50);
        this->RPiIP = model.value("RPiIP", "192.168.1.100");
        this->RPiPort = model.value("RPiPort", 8080);
        string posLabel[] = {"x", "y", "z", "roll", "pitch", "yaw"};

        // Interate through arrays
        for (int i = 0; i < 6; i++)
        {
            this->homePos[i] = model["homePos"][posLabel[i]];
            this->endEffectorPosLimit[i] = (float)model["endEffectorPosLimit"][posLabel[i]];
            this->brickPickUpPos[i] = model["brickPickUpPos"][posLabel[i]];
        }
        for (int i = 0; i < cableMotorNum; i++)
        {
            frmOutUnitV[i] << 0, 0, model["pulleyZdir"][i];
            for (int j = 0; j < 3; j++)
            { // for each xyz coordinates
                this->frmOut[i][j] = model["frameAttachments"][i][j];
                this->endOut[i][j] = model["endEffectorAttachments"][i][j];
            }
        }
        if (this->useIndividualCableScale)
        {
            if (model.contains("cableMotorScaleIndividual"))
            {
                for (int i = 0; i < this->cableMotorNum; i++)
                {
                    this->cableMotorScaleIndividual[i] = (float)model["cableMotorScaleIndividual"][i];
                }
            }
        }
        else
        {
            for (int i = 0; i < this->cableMotorNum; i++)
            {
                this->cableMotorScaleIndividual[i] = this->cableMotorScale;
            }
        }
        if (!this->isOnline)
            cout << "!!!!!Warning!!!!!Offline Mode. Modify the config file if you want go online" << endl;
        if (reconnect)
            this->Connect();
        this->isValidModel = true;
    }
    catch (const exception &e)
    {
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
        cout << "Error: Cannot read robot config file. Exit..." << endl;
        this->isValidModel = false;
    }
}

void Robot::UpdatePos(double pos[6]){
    for(int i = 0; i < 6; i++){
        this->endEffectorPos[i] = pos[i];
    }
    
    vector<double> cableLengthList = this->GetCableLength();
    // calibration motors
    if(this->isOnline){
        // cable motors
        cout << "Calibrating Cable Motor" << endl;
        for(int index = 0; index < this->cableMotorNum; index++){
            this->cable.CalibrationMotor(index, this->CableMotorLengthToCmd(index, cableLengthList[index]));
        }
    }
    this->PrintEEPos();
    // for teknic motors
    if(this->useCableMotor){
        cout << "Cable Motor length / internal counts: " << endl;
        for (int i = 0; i < this->cableMotorNum; i++){
            cout << "\t" << "Cable " << i << ": " << cableLengthList[i] << " / " << this->cable.GetMotorPosMeasured(i)  << "    " << endl;
        }

    }
}

void Robot::UpdatePosFromFile(string filename, bool calibration){
    try{
        ifstream file (filename); //"lastPos.txt" or "currentPos.csv"
        if(!file.good()){
            cout << "Error: File Not Exist. Keep using current pos." << endl;
            return;
        }
        string temp;
        int count = 0;
        if (file.is_open())
        {
            try
            {
                while (file >> temp) // read the file cell by cell
                {
                    if (count < 6)
                    {
                        this->endEffectorPos[count] = stod(temp);
                    } // convert string to double stod() for the first 6 inputs
                    else if (count < 10)
                    {
                        this->railOffset[count - 6] = stod(temp);
                    } // reading the rail offset, then break while loop
                    else
                    {
                        break;
                    }
                    count++;
                }
                cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
            }
            catch (int e)
            {
                cout << "Check if currentPos.csv matches the in1 input no." << endl;
            }
            vector<double> cableLengthList = this->GetCableLength();
            // calibration motors
            if (this->isOnline && calibration)
            {
                // cable motors
                cout << "Calibrating Cable Motor" << endl;
                for (int index = 0; index < this->cableMotorNum; index++)
                {
                    this->cable.CalibrationMotor(index, this->CableMotorLengthToCmd(index, cableLengthList[index]));
                }
                // slider motors
                // cout << "Calibrating Rail Motor" << endl;
                // for(int index = 0; index < this->railMotorNum; index++){
                //     this->rail.CalibrationMotor(index, this->RailMotorLengthToCmd(index, railOffset[index]));
                // }
                // cout << "Updating motor counts completed" << endl;
            }
            this->PrintEEPos();
            // for teknic motors
            if (this->useCableMotor)
            {
                cout << "Cable Motor length / internal counts: " << endl;
                for (int i = 0; i < this->cableMotorNum; i++)
                {
                    cout << "\t" << "Cable " << i << ": " << cableLengthList[i] << " / " << this->cable.GetMotorPosMeasured(i) << "    " << endl;
                }
            }
            // for twincat motors
            if (this->useRailMotor)
            {
                cout << "Rail Motor length / internal counts: " << endl;
                vector<int> railMotorCmd = this->rail.GetMotorPosMeasured();
                for (int i = 0; i < this->railMotorNum; i++)
                {
                    cout << "\t" << "Rail " << i << ": " << this->railOffset[i] << " / " << railMotorCmd[i] << "    " << endl;
                }
            }

            if (this->isOnline)
                this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
        }
    }
    catch (const exception &e)
    {
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
    }
}

void Robot::UpdatePosFromLog(string filename)
{
    try
    {
        ifstream file(filename); //"lastPos.txt" or "currentPos.csv"
        if (!file.good())
        {
            cout << "Error: File Not Exist. Keep using current pos." << endl;
            return;
        }
        string temp;
        int count = 0;
        if (file.is_open())
        {
            try
            {
                while (file >> temp)
                {
                    if (count < 6)
                    {
                        this->endEffectorPos[count] = stod(temp);
                    } // convert string to double stod() for the first 6 inputs
                    else if (count < 10)
                    {
                        this->railOffset[count - 6] = stod(temp);
                    } // reading the rail offset, then break while loop
                    else
                    {
                        break;
                    }
                    count++;
                }
                cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
            }
            catch (int e)
            {
                cout << "Check if currentPos.csv matches the in1 input no." << endl;
            }
            vector<double> cableLengthList = this->GetCableLength();
            // calibration motors
            if (this->isOnline)
            {
                // cable motors
                cout << "Calibrating Cable Motor" << endl;
                for (int index = 0; index < this->cableMotorNum; index++)
                {
                    this->cable.CalibrationMotor(index, this->CableMotorLengthToCmd(index, cableLengthList[index]));
                }
                // slider motors
                cout << "Calibrating Rail Motor" << endl;
                for (int index = 0; index < this->railMotorNum; index++)
                {
                    this->rail.CalibrationMotor(index, this->RailMotorLengthToCmd(index, railOffset[index]));
                }
                cout << "Updating motor counts completed" << endl;
            }
            this->PrintEEPos();
            // for teknic motors
            cout << "Cable Motor length / internal counts: " << endl;
            for (int i = 0; i < this->cableMotorNum; i++)
            {
                cout << "\t" << "Cable " << i << ": " << cableLengthList[i] << " / " << this->cable.GetMotorPosMeasured(i) << "    " << endl;
            }
            // for twincat motors
            cout << "Rail Motor length / internal counts: " << endl;
            vector<int> railMotorCmd = this->rail.GetMotorPosMeasured();
            for (int i = 0; i < this->railMotorNum; i++)
            {
                cout << "\t" << "Rail " << i << ": " << this->railOffset[i] << " / " << railMotorCmd[i] << "    " << endl;
            }

            if (this->isOnline)
                this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
        }
    }
    catch (const exception &e)
    {
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
    }
}

void Robot::PrintRobotConfig()
{

    // print to screen
    cout << "Cable Motor Number:\t\t" << this->cableMotorNum << endl;
    cout << "Rail Motor Number:\t\t" << this->railMotorNum << endl;
    cout << "End-Effector to Ground Offset:\t" << this->endEffToGroundOffset << endl;
    cout << "Absolute Torque Limit\t\t" << this->absTrqLmt << endl;
    cout << "Target Torque\t\t\t" << this->targetTrq << endl;
    cout << "Cable Motor Scale\t\t" << this->cableMotorScale << endl;
    cout << "Rail Motor Scale\t\t" << this->railMotorScale << endl;
    cout << "Pully Radius\t\t\t" << this->pulleyRadius << endl;
    cout << "Rotational Angle Offset\t\t" << this->rotationalAngleOffset << endl;
    cout << "Rotational Distance Offset\t" << this->rotationalDistanceOffset << endl;
    cout << "Gripper Comm Port\t\t" << this->gripperCommPort << endl;
    cout << "Rail Brake Comm Port\t\t" << this->railBrakeCommPort << endl;

    cout << "End Effector Pos Limit: " << endl;
    for (int i = 0; i < 3; i++)
    {
        cout << "\t" << posLabel[i] << ": (" << this->endEffectorPosLimit[i * 2] << " ~ " << this->endEffectorPosLimit[i * 2 + 1] << ")" << endl;
    }

    cout << "Home Pos: " << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->homePos[i] << endl;
    }

    cout << "Brick Pick Up Pos: " << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "\t" << posLabel[i] << ": " << this->brickPickUpPos[i] << endl;
    }
}

vector<double> Robot::EEPoseToCableLength(double eePos[], double railOffset[])
{ // given an EE pose, calculate the EE to pulley cable lengths
    vector<double> result;
    // local variables
    Vector3d orB[8]; // vector from frame 0 origin to end effector cable outlet
    Vector3d orA[8]; // vector from frame 0 origin to cable outlet point on rotating pulley
    Matrix3d oe_R;   // rotation matrix from end effector to frame 0
    Matrix3d Ra, Rb, Rc;
    double a = eePos[3], b = eePos[4], c = eePos[5];

    Ra << 1, 0, 0,
        0, cos(a), -sin(a),
        0, sin(a), cos(a);
    Rb << cos(b), 0, sin(b),
        0, 1, 0,
        -sin(b), 0, cos(b);
    Rc << cos(c), -sin(c), 0,
        sin(c), cos(c), 0,
        0, 0, 1;
    oe_R = Ra * Rb * Rc;                           // Rotation matrix from given pose rotation
    Vector3d endEfr(eePos[0], eePos[1], eePos[2]); // Translation of end-effector
    for (int i = 0; i < cableMotorNum; i++)
    {
        ///// Calculate orB[8] vectors /////
        orB[i] = oe_R * endOut[i] + endEfr;

        ///// Calculate cable outlet point on rotating pulley, ie point A /////
        Vector3d plNormal = frmOutUnitV[i].cross(orB[i] - frmOut[i]); // normal vector of the plane that the rotating pulley is in
        Vector3d VecC = plNormal.cross(frmOutUnitV[i]);               // direction from fixed point towards pulley center
        Vector3d zOffset(0, 0, 0);                                    // set a z offset for frame out for pulleys on linear rails
        if (i < 4)
        {
            zOffset << 0, 0, railOffset[i];
        }
        Vector3d orC = VecC / VecC.norm() * pulleyRadius + frmOut[i] + zOffset; // vector from frame 0 origin to rotating pulley center
        double triR = pulleyRadius / (orB[i] - orC).norm();                     // triangle ratio??
        orA[i] = orC + triR * triR * (orB[i] - orC) - triR * sqrt(1 - triR * triR) * ((plNormal / plNormal.norm()).cross(orB[i] - orC));

        ///// Calculate arc length /////
        Vector3d UVecCF = -VecC / VecC.norm();                    // unit vector of CF
        Vector3d UVecCA = (orA[i] - orC) / (orA[i] - orC).norm(); // unit vector of CA
        double l_arc = pulleyRadius * acos(UVecCF.dot(UVecCA));

        ///// Sum the total cable length /////
        result.push_back(i < 4 ? l_arc + (orA[i] - orB[i]).norm() - railOffset[i] : l_arc + (orA[i] - orB[i]).norm()); // WHY minus railOffset again?
        // lengths[i] = i < 4 ? l_arc + (orA[i] - orB[i]).norm() - railOffset[i] : l_arc + (orA[i] - orB[i]).norm(); // Need to subtract the rail offset for the bottom 4 motors
    }
    // copy(railOffset, railOffset+4, lengths+cableMotorNum); // assign array elements after motor numbers as rail lengths // rail_offset default size of 4
    return result;
}

vector<double> Robot::EEPoseToCableLength(double eePos[])
{
    return this->EEPoseToCableLength(eePos, this->railOffset);
}

vector<double> Robot::EEPoseToCableLength(vector<double> eePosVector)
{
    return this->EEPoseToCableLength(eePosVector, this->railOffset);
}

vector<double> Robot::EEPoseToCableLength(vector<double> eePosVector, double railOffset[])
{
    double *eePosArray = &eePosVector[0];
    return this->EEPoseToCableLength(eePosArray, railOffset);
}

double Robot::GetEEToGroundOffset() { return endEffToGroundOffset; }

float Robot::GetWorkingTrq() { return targetTrq; }

float Robot::GetAbsTrqLmt() { return absTrqLmt; }

int Robot::GetCableMotorNum() { return cableMotorNum; }

int Robot::GetRailMotorNum() { return railMotorNum; }

string Robot::GetGripperCommPort() { return gripperCommPort; }

string Robot::GetRailBrakeCommPort() { return railBrakeCommPort; }

string Robot::GetCableBrakeCommPort() { return cableBrakeCommPort; }

double Robot::GetCableMotorScale(int motorID) { return cableMotorScaleIndividual[motorID]; }

double Robot::GetRailMotorScale() { return railMotorScale; }

float Robot::GetEffVelLmt() { return endEffVelLmt; }

int Robot::GetCableMotorBrakeNum() { return cableMotorBrakeNum; }


int32_t Robot::CableMotorLengthToCmd(int motorID, double length)
{
    return length * cableMotorScaleIndividual[motorID];
}

double Robot::CableMotorCmdToLength(int motorID, int cmd)
{
    return cmd / cableMotorScaleIndividual[motorID];
}

int32_t Robot::RailMotorLengthToCmd(int motorID, double length)
{
    return length * railMotorScale;
}

double Robot::RailMotorCmdToLength(int motorID, int cmd)
{
    return cmd / railMotorScale;
}

vector<double> Robot::GetCableLength()
{
    return this->EEPoseToCableLength(this->endEffectorPos);
};

vector<int32_t> Robot::EEPoseToCmd(vector<double> eePos)
{
    return EEPoseToCmd(eePos, this->railOffset);
}

vector<int32_t> Robot::EEPoseToCmd(vector<double> eePosVector, double railOffset[])
{
    double *eePosArray = &eePosVector[0];
    return this->EEPoseToCmd(eePosArray, railOffset);
}

vector<int32_t> Robot::EEPoseToCmd(double eePos[])
{
    return EEPoseToCmd(eePos, this->railOffset);
}

vector<int32_t> Robot::EEPoseToCmd(double eePos[], double railOffset[])
{
    vector<int32_t> result;
    vector<double> lengthList = this->EEPoseToCableLength(eePos, railOffset);
    int index = 0;
    for (double length : lengthList)
    {
        // result.push_back(this->CableMotorLengthToCmd(index, length));
        result.push_back(this->CableMotorLengthToCmd(index, length));
        index++;
    }
    return result;
};

bool Robot::eBrake(bool cableBrake, bool railBrake)
{
    if (!useEBrake)
        return true;
    if (kbhit())
    { // Emergency quit during trajectory control
        this->cable.StopAllMotor();
        if (cableBrake)
            this->brake.CloseAllCableBrake();
        if (railBrake)
            this->brake.CloseAllRailBrake();
        string userInput;
        cout << endl;
        cout << "================================== Trajectory Stopped!! ==================================" << endl;
        system("pause");
        return false;
    }
    return true;
};

vector<vector<int32_t>> Robot::PoseTrajToCmdTraj(vector<vector<double>> posTraj)
{
    vector<vector<int32_t>> result;
    for (vector<double> pos : posTraj)
    {
        result.push_back(this->EEPoseToCmd(pos));
    }
    return result;
}

bool Robot::RunCableTraj(vector<vector<double>> posTraj, bool showAtten)
{
    vector<vector<int32_t>> cmdTraj = this->PoseTrajToCmdTraj(posTraj);
    int index = 0;
    if (showAtten)
    {
        for (int i = 0; i < 18; i++)
        {
            cout << endl;
        }
    }
    // check every step to make sure the command is not too large
    vector<int32_t> lastFrameCmd = {0, 0, 0, 0, 0, 0, 0, 0};
    vector<double> lastFrameLength = this->EEPoseToCableLength(this->endEffectorPos);

    for (int i = 0; i < 8; i++)
    {
        lastFrameCmd[i] = lastFrameLength[i] * this->cableMotorScaleIndividual[i];
    }
    cout << "Last Frame Length: " << lastFrameLength[0] << " " << lastFrameLength[1] << " " << lastFrameLength[2] << " " << lastFrameLength[3] << " " << lastFrameLength[4] << " " << lastFrameLength[5] << " " << lastFrameLength[6] << " " << lastFrameLength[7] << endl;
    cout << "Last Frame Cmd: " << lastFrameCmd[0] << " " << lastFrameCmd[1] << " " << lastFrameCmd[2] << " " << lastFrameCmd[3] << " " << lastFrameCmd[4] << " " << lastFrameCmd[5] << " " << lastFrameCmd[6] << " " << lastFrameCmd[7] << endl;
    vector<double> lastFrameEEPos = {this->endEffectorPos[0], this->endEffectorPos[1], this->endEffectorPos[2]};
    for (int j = 0; j < posTraj.size(); j++)
    {
        vector<int32_t> frame = cmdTraj[j];
        for (int i = 0; i < 8; i++)
        {
            if ((frame[i] - lastFrameCmd[i]) / this->cableMotorScaleIndividual[i] > 0.5 || (frame[i] - lastFrameCmd[i]) / this->cableMotorScaleIndividual[i] < -0.5)
            {
                cout << "================================== Trajectory Stopped!! ==================================" << endl;
                cout << "Motor " << i << endl;
                cout << "Frame " << index << endl;
                cout << "Step too large! range should be in -0.5m(" << -0.5 * this->cableMotorScaleIndividual[i] << ") ~ 0.5m(" << 0.5 * this->cableMotorScaleIndividual[i] << ")." << endl;
                cout << "Current command: " << frame[i] << " Length: " << frame[i] / this->cableMotorScaleIndividual[i] << endl;
                cout << "Last frame command: " << lastFrameCmd[i] << " Length: " << lastFrameCmd[i] / this->cableMotorScaleIndividual[i] << endl;
                cout << "Difference: " << (frame[i] - lastFrameCmd[i]) / this->cableMotorScaleIndividual[i] << "m" << endl;
                cout << "Last frame EE Pos: x" << lastFrameEEPos[0] << " y" << lastFrameEEPos[1] << " z" << lastFrameEEPos[2] << endl;
                cout << "Cmd EE Pos: x" << posTraj[index][0] << " y" << posTraj[index][1] << " z" << posTraj[index][2] << endl;
                return false;
            }
            lastFrameCmd[i] = frame[i];
        }
        lastFrameEEPos[0] = posTraj[j][0];
        lastFrameEEPos[1] = posTraj[j][1];
        lastFrameEEPos[2] = posTraj[j][2];
    }

    for (vector<int32_t> frame : cmdTraj)
    {
        auto start = chrono::steady_clock::now();
        if (!this->eBrake(true, false))
            return false;
        if (!this->cable.MoveAllMotorCmd(frame))
        {
            cout << "Trajectory stopped" << endl;
            return false;
        }
        for (int i = 0; i < 6; i++)
            this->endEffectorPos[i] = posTraj[index][i];
        index++;
        if (this->isOnline)
            this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
        if (showAtten)
        {
            printf("\x1b[18A");
            cout << "Current frame: " << index << endl;
            this->PrintEEPos();
            cout << "Current Cable Motor Count: " << endl;
            for (int i = 0; i < this->cableMotorNum; i++)
            {
                cout << "\tCable " << i << ": " << frame[i] << endl;
            }
            // cout << "Current Measured Cable Motor Trq: " << endl;
            // for (int i = 0; i < this->cableMotorNum; i++){
            //     cout << "\tCable " << i << ": " << this->cable.GetMotorTorqueMeasured(i) << endl;
            // }
        }
        auto end = chrono::steady_clock::now();
        double dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end - start).count() - 1;
        cout << "dif: " << dif << endl;
        while (dif > 0)
        {
            end = chrono::steady_clock::now();
            dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end - start).count() - 1;
        }
    }
    return true;
}

bool Robot::MoveToLinear(double dest[], int time, bool showAtten, bool useEBrake)
{
    if (!useEBrake)
        this->useEBrake = false;
    bool result = this->RunCableTraj(GenLinearTrajForCableMotor(this->endEffectorPos, dest, time, showAtten), showAtten);
    this->useEBrake = true;
    return result;
}

bool Robot::MoveToParaBlend(double dest[], int time, bool showAtten)
{
    return this->RunCableTraj(GenParaBlendTrajForCableMotor(this->endEffectorPos, dest, time), showAtten);
}

bool Robot::MoveToParaBlend(double dest[], bool showAtten)
{
    float time = sqrt(pow(dest[0] - this->endEffectorPos[0], 2) + pow(dest[1] - endEffectorPos[1], 2) + pow(dest[2] - endEffectorPos[2], 2)) / this->endEffVelLmt * 1000;
    return this->RunCableTraj(GenParaBlendTrajForCableMotor(this->endEffectorPos, dest, time), showAtten);
}


void Robot::RaiseRailWithCableByLengthAbsulote(int railIndex, int cableIndex, float target)
{
    if (target < 0 || target > 2.55)
    {
        cout << "WARNING! Intended rail count is out of bound! " << target << endl;
        cout << "Valid range: " << 0 << " ~ " << 2.55 << endl;
        if(!CheckContinue()){
                return;
        }
    }
    this->brake.OpenCableBrakeByIndex(cableIndex);
    this->brake.OpenRailBrakeByIndex(railIndex);

    double dura = abs(target - this->railOffset[railIndex]) / this->railVelLmt * 1000; // *1000 to change unit to ms
    if (dura <= 200 || dura > 10000)
    {
        cout << "Invalid duration: " << dura << " ms" << endl;
        if(!CheckContinue()){
            return;
        }
    } // Don't run traj for incorrect timing

    static double a, b, c; // coefficients for cubic spline trajectory
    cout << "ATTENTION: Raise rail function is called. Estimated time: " << dura / 1000 << " seconds" << endl;
    cout << "Generating raise rail trajectory..." << endl;

    // Solve coefficients of equations for cubic
    a = this->railOffset[railIndex];
    b = 3 / (dura * dura) * (target - this->railOffset[railIndex]);
    c = -2 / (dura * dura * dura) * (target - this->railOffset[railIndex]);

    // Generate rail and cable traj.
    long nErr;
    double step{};
    double t = 0;
    long dur = 0;
    double railOffset[6] = {0};
    vector<double> cableLength;
    vector<int32_t> cableTraj;
    vector<int32_t> railTraj;
    while (t <= dura)
    {
        railOffset[railIndex] = a + b * t * t + c * t * t * t; // update new rail offset
        // cout << "Rail offset: " << railOffset[railIndex] << endl;
        int32_t cableFrame = this->EEPoseToCmd(this->endEffectorPos, railOffset)[cableIndex];
        cableTraj.push_back(cableFrame);
        int32_t railFrame = this->RailMotorLengthToCmd(railIndex, railOffset[railIndex]);
        railTraj.push_back(railFrame);
        t += MILLIS_TO_NEXT_FRAME;
    }

    cout << "Raise rail trajectory Generated" << endl;
    for (int i = 0; i < cableTraj.size(); i++)
    {
        if (!this->eBrake(true, true))
            return;
        auto start = chrono::steady_clock::now();
        rail.SelectWorkingMotor(railIndex);
        rail.MoveSelectedMotorCmdAbsulote(railTraj[i]);
        cout << "Frame " << i << " Rail " << railIndex << " Cmd: " << railTraj[i] << "(" << this->RailMotorCmdToLength(railIndex, railTraj[i]) << "m)" 
                              << " Cable " << cableIndex << " Cmd: " << cableTraj[i] << "(" << this->CableMotorCmdToLength(cableIndex, cableTraj[i]) << "m)" << endl;
        if (!cable.MoveSingleMotorCmd(cableIndex, cableTraj[i], true))
        {
            cout << "Move cable " << cableIndex << " failed!";
            break;
        }
        this->railOffset[railIndex] = this->RailMotorCmdToLength(railIndex, railTraj[i]);
        if (this->isOnline)
            this->posLogger.LogPos(this->endEffectorPos, this->railOffset);
        dur = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        double dif = MILLIS_TO_NEXT_FRAME - dur - 1;
        if (dif > 0)
        {
            Sleep(dif);
        }
    }

    // Wait for rail motor to reach intented position before enabling brake again
    // while(posNow[railIndex]>step+2 || posNow[railIndex]<step-2){
    //     nErr = AdsSyncReadReq(pAddr, ADSIGRP_SYM_VALBYHND, hdlList[2], sizeof(posNow), &posNow[0]); // read "MAIN.AxisList[railIndex].NcToPlc.ActPos"
    //     if (nErr) { cout << "Error: Rail[" << id << "] AdsSyncReadReq: " << nErr << '\n'; break; }
    //     // cout << posNow[railIndex] << endl;
    // }
    Sleep(50);
    cout << "Rail traj done .... \n";

    this->brake.CloseCableBrakeByIndex(cableIndex);
    this->brake.CloseRailBrakeByIndex(railIndex);
}

void Robot::SavePosToFile(string filename)
{
    if (!this->isOnline)
        return;

    //// Safe system shut down, safe last pos and emegency shut down
    // Saving last position before quiting programme
    cout << "Saving last position to file: " << filename << endl;
    ofstream myfile;
    myfile.open(filename);
    for (int i = 0; i < 6; i++)
    {
        myfile << this->endEffectorPos[i] << " ";
    }
    myfile << endl;

    for (int i = 0; i < this->railMotorNum; i++)
    {
        myfile << this->railOffset[i] << " ";
    }
    myfile << endl;

    for (int i = 0; i < this->cableMotorNum; i++)
    {
        myfile << this->cable.GetMotorPosMeasured(i) << " ";
    }
    vector<int> railMotorCmd = this->rail.GetMotorPosMeasured();
    for (int i = 0; i < this->railMotorNum; i++)
    {
        if (this->useRailMotor)
        {
            myfile << railMotorCmd[i] << " ";
        }
        else
        {
            myfile << 0 << " ";
        }
    }

    myfile << endl;
}