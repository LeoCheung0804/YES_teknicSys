#include "..\include\Robot.h"
#include "..\tools\json.hpp"
#include "..\Dependencies\eigen-3.3.7\Eigen\Dense"
#include "..\include\TrajectoryGenerator.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>

using json = nlohmann::json;

Robot::Robot(){};

// Constructor, based on the requested model
Robot::Robot(string robotConfigPath){
    this->UpdateModelFromFile(robotConfigPath);
    

}

void Robot::Connect(){

    // Init BLE Nodes
    this->gripper = GripperController(this->isOnline);
    this->gripper.Connect(this->gripperCommPort);

    // Init Rail Motor Nodes
    this->rail = RailController(this->isOnline);
    this->rail.Connect(851, this->railMotorNum, this->railBreakCommPort);
    
    // Init Cable Motor Nodes
    this->cable = CableController(this->cableMotorNum, this->cableMotorBrakeNum, this->cableBreakCommPort, this->isOnline);
    
    cout << "All motors, all brakes and gripper connected success." << endl;
}

void Robot::Disconnect(){
    this->gripper.Disconnect();

}
const double Robot::defaultRailOffset[4] = {0, 0, 0, 0}; // default all zeros

bool Robot::IsValid(){ 
    return this->isValidModel; 
    }

bool Robot::CheckLimits(){
    for (int i = 0; i < 3; i++){
        if(this->endEffectorPos[i]<this->endEffectorPosLimit[i*2]) return false;
        if(this->endEffectorPos[i]>this->endEffectorPosLimit[i*2+1]) return false;
    }
    return true;
}

void Robot::PrintEEPos(){ 
    cout << "Current EE Pos: " << endl;
    for (int i = 0; i < 6; i++){
        cout << "\t" << posLabel[i] << ": " << this->endEffectorPos[i] << "    " << endl;
    }
}

void Robot::PrintBrickPickUpPos(){ 
    cout << "Brick Pick Up Pos: ";
    for (int i = 0; i < 6; i++){
        cout << "\t" << posLabel[i] << ": " << this->brickPickUpPos[0] << "    " << endl;
    }
}

void Robot::PrintRailOffset(){ // Print railOffset[]
    cout << "Rail Offset: " << endl;
    for (int i = 0; i < this->railMotorNum; i++){
        cout << "\t" << "Cable " << i << ": " << this->railOffset[i]  << "    " << endl;
    }
}

void Robot::PrintCableLength(){ 
    cout << "Cable Length: ";
    vector<double> cableLength = this->EEPoseToCableLength(this->endEffectorPos);
    for(int i = 0; i < this->cableMotorNum; i++){ cout <<  cableLength[i] << " "; }
    cout << endl;
    cout << "Rail Offset: ";
    for(int i = 0; i < this->railMotorNum; i++){ cout << this->railOffset[i] << " "; }
    cout << endl;
}

void Robot::PrintHomePos(){ // Print home[]
    cout << "HOME Pos: ";
    for(int i = 0; i < 6; i++){ cout << homePos[i] << " "; }
    cout << endl;
}

void Robot::UpdateModelFromFile(string filename){ // Read model.json file
    json model;
    try{
        ifstream file(filename, ifstream::binary);
        file >> model;
        cout << "Importing " << filename << ": " << model.value("ModelName", "NOT FOUND") << endl;
        
        this->isOnline = model.value("isOnline", false);

        // Initialize parameters
        this->cableMotorNum = model.value("cableMotorNum", 8); // Default value set in 2nd input
        this->railMotorNum = model.value("railMotorNum", 4);
        this->absTrqLmt = model.value("absTorqueLmt", 60);
        this->endEffToGroundOffset = model.value("endEffToGroundOffset", -0.28);
        this->targetTrq = model.value("targetTorque", -2.5);
        this->cableMotorScale = model.value("cableMotorScale", 509295);
        this->railMotorScale = model.value("railMotorScale", 38400000);
        this->pulleyRadius = model.value("pulleyRadius", 0.045);
        this->rotationalAngleOffset = model.value("rotationalAngleOffset", 21.8);
        this->rotationalDistanceOffset = model.value("rotationalDistanceOffset", -0.0075);
        this->gripperCommPort = model.value("gripperCommPort", "\\\\.\\COM26");
        this->railBreakCommPort = model.value("railBreakCommPort", "\\\\.\\COM26");
        this->brickPickUpOffset = model.value("brickPickUpOffset", 0.12);
        this->velLmt = model.value("velLmt", 0.45);
        string posLabel[] = {"x", "y", "z", "yaw", "pitch", "roll"};

        // Interate through arrays
        for(int i=0; i<6; i++){
            this->homePos[i] = model["homePos"][posLabel[i]];
            this->endEffectorPosLimit[i] = (float) model["endEffectorPosLimit"][posLabel[i]];
            this->brickPickUpPos[i] = model["brickPickUpPos"][posLabel[i]];
        }
        for(int i=0; i<cableMotorNum; i++){
            frmOutUnitV[i] << 0, 0, model["pulleyZdir"][i];
            for(int j=0; j<3; j++){ // for each xyz coordinates
                this->frmOut[i][j] = model["frameAttachments"][i][j];
                this->endOut[i][j] = model["endEffectorAttachments"][i][j];
            }
        }
        
        Robot::PrintRobotConfig();
        if(!this->isOnline)
            cout << "!!!!!Warning!!!!!Offline Mode. Modify the config file if you want go online" << endl;
    }
    catch(const exception& e){
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
        cout << "Error: Cannot read robot config file. Exit..." << endl;
        exit(-1);
    }
}

void Robot::UpdatePosFromFile(string filename){
    try{
        ifstream file (filename); //"lastPos.txt" or "currentPos.csv"
        string temp;
        int count = 0;
        if(file.is_open()){
            try{
                while (file >> temp){
                    if(count < 6){ this->endEffectorPos[count] = stod(temp); } // convert string to double stod() for the first 6 inputs
                    else if(count < 10){ this->railOffset[count-6] = stod(temp); } // reading the rail offset, then break while loop
                    else { break; }
                    count++;
                }
                cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
            }
            catch(int e){ cout << "Check if currentPos.csv matches the in1 input no." << endl; }
            // calibration motors
            // cable motors
            vector<double> cableLengthList = this->GetCableLength();
            if(this->isOnline){
                for(int index = 0; index < this->cableMotorNum; index++){
                    this->cable.CalibrationMotor(index, this->CableMotorLengthToCmd(index, cableLengthList[index]));
                }
                // slider motors
                for(int index = 0; index < this->railMotorNum; index++){
                    this->rail.CalibrationMotor(index, this->RailMotorOffsetToCmd(index, railOffset[index]));
                }
            }
            cout << "Updating motor counts completed" << endl;
            this->PrintEEPos();
            // for teknic motors
            cout << "Cable Motor length / internal counts: " << endl;
            for (int i = 0; i < this->cableMotorNum; i++){
                cout << "\t" << "Cable " << i << ": " << cableLengthList[i] << " / " << this->cable.GetMotorPosMeasured(i)  << "    " << endl;
            }
            // for twincat motors
            cout << "Rail Motor length / internal counts: " << endl;
            vector<int> railMotorCmd = this->rail.GetMotorPosMeasured();
            for(int i = 0; i < this->railMotorNum; i++){
                cout << "\t" << "Rail " << i << ": " << this->railOffset[i] << " / " << railMotorCmd[i]  << "    " << endl;
            }
        }
    }
    catch(const exception& e){
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
    }
}

void Robot::PrintRobotConfig(){
    
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
        cout << "Rail Break Comm Port\t\t" << this->railBreakCommPort << endl;

        cout << "End Effector Pos Limit: " << endl;
        for(int i = 0; i < 3; i++){
            cout << "\t" << posLabel[i] << ": (" << this->endEffectorPosLimit[i*2] << " ~ " << this->endEffectorPosLimit[i*2+1] << ")" << endl;
        }

        cout << "Home Pos: " << endl;
        for (int i = 0; i < 6; i++){
            cout << "\t" << posLabel[i] << ": " << this->homePos[i] << endl;
        }

        cout << "Brick Pick Up Pos: " << endl;
        for (int i = 0; i < 6; i++){
            cout << "\t" << posLabel[i] << ": " << this->brickPickUpPos[i] << endl;
        }
}

vector<double> Robot::EEPoseToCableLength(double eePos[]){ // given an EE pose, calculate the EE to pulley cable lengths
    vector<double> result;
    // local variables
    Vector3d orB[8]; // vector from frame 0 origin to end effector cable outlet
    Vector3d orA[8]; // vector from frame 0 origin to cable outlet point on rotating pulley
    Matrix3d oe_R; // rotation matrix from end effector to frame 0
    Matrix3d Ra, Rb, Rc;
    double a = eePos[3], b = eePos[4], c = eePos[5];
    
    Ra << 1, 0, 0,
          0, cos(a), -sin(a),
          0, sin(a), cos(a);
    Rb << cos(b), 0,  sin(b),
          0, 1, 0,
          -sin(b), 0, cos(b);
    Rc << cos(c), -sin(c), 0, 
          sin(c), cos(c), 0, 
          0, 0, 1;
    oe_R = Ra * Rb * Rc; // Rotation matrix from given pose rotation
    Vector3d endEfr(eePos[0], eePos[1], eePos[2]); // Translation of end-effector
    for(int i = 0; i < cableMotorNum; i++){
        ///// Calculate orB[8] vectors /////
        orB[i] = oe_R*endOut[i] + endEfr;
                
        ///// Calculate cable outlet point on rotating pulley, ie point A /////
        Vector3d plNormal = frmOutUnitV[i].cross(orB[i] - frmOut[i]); // normal vector of the plane that the rotating pulley is in
        Vector3d VecC = plNormal.cross(frmOutUnitV[i]); // direction from fixed point towards pulley center
        Vector3d zOffset(0, 0, 0); // set a z offset for frame out for pulleys on linear rails
        if(i < 4){ zOffset << 0, 0, this->railOffset[i]; }
        Vector3d orC = VecC/VecC.norm()*pulleyRadius + frmOut[i] + zOffset; // vector from frame 0 origin to rotating pulley center
        double triR = pulleyRadius / (orB[i] - orC).norm(); // triangle ratio??
        orA[i] = orC + triR*triR*(orB[i] - orC) - triR*sqrt(1 - triR*triR)*((plNormal/plNormal.norm()).cross(orB[i] - orC));
        
        ///// Calculate arc length /////
        Vector3d UVecCF = -VecC / VecC.norm(); // unit vector of CF
        Vector3d UVecCA = (orA[i]- orC) / (orA[i]- orC).norm(); // unit vector of CA
        double l_arc = pulleyRadius * acos(UVecCF.dot(UVecCA));

        ///// Sum the total cable length /////
        result.push_back(i < 4 ? l_arc + (orA[i] - orB[i]).norm() - this->railOffset[i] : l_arc + (orA[i] - orB[i]).norm()); // WHY minus railOffset again?
        // lengths[i] = i < 4 ? l_arc + (orA[i] - orB[i]).norm() - railOffset[i] : l_arc + (orA[i] - orB[i]).norm(); // Need to subtract the rail offset for the bottom 4 motors 
    }
    //copy(railOffset, railOffset+4, lengths+cableMotorNum); // assign array elements after motor numbers as rail lengths // rail_offset default size of 4
    return result;
}

vector<double> Robot::EEPoseToCableLength(vector<double> eePosVector){
    double* eePosArray = &eePosVector[0];
    return this->EEPoseToCableLength(eePosArray);
}

double Robot::GetEEToGroundOffset(){ return endEffToGroundOffset; }

float Robot::GetWorkingTrq(){ return targetTrq; }

float Robot::GetAbsTrqLmt(){ return absTrqLmt; }

int Robot::GetCableMotorNum(){ return cableMotorNum; }

int Robot::GetRailMotorNum(){ return railMotorNum; }

string Robot::GetGripperCommPort(){ return gripperCommPort; }

string Robot::GetRailBreakCommPort(){ return railBreakCommPort; }

string Robot::GetCableBreakCommPort(){ return cableBreakCommPort; }

int32_t Robot::GetCableMotorScale(){ return cableMotorScale; }

int32_t Robot::GetRailMotorScale(){ return railMotorScale; }

float Robot::GetVelLmt(){ return velLmt; }

int Robot::GetCableMotorBreakNum(){ return cableMotorBrakeNum; }

int32_t Robot::CableMotorLengthToCmdAbsulote(double length){
    return length * cableMotorScale;
}

int32_t Robot::CableMotorLengthToCmd(int motorID, double length){
    return (length - this->robotOffset[motorID]) * cableMotorScale;
}

int32_t Robot::RailMotorOffsetToCmd(int motorID, double length){
    return (length - this->robotOffset[motorID]) * railMotorScale; 
}

vector<double> Robot::GetCableLength(){
    return this->EEPoseToCableLength(this->endEffectorPos);
};

vector<int32_t> Robot::EEPoseToCmd(vector<double> eePos){
    vector<int32_t> result;
    vector<double> lengthList = this->EEPoseToCableLength(eePos);
    int index = 0;
    for(double length : lengthList){
        //result.push_back(this->CableMotorLengthToCmd(index, length));
        result.push_back(this->CableMotorLengthToCmdAbsulote(length));
        index++;
    }
    return result;
}

vector<vector<int32_t>> Robot::PoseTrajToCmdTraj(vector<vector<double>> posTraj){
    vector<vector<int32_t>> result;
    for(vector<double> pos : posTraj){
        result.push_back(this->EEPoseToCmd(pos));
    }
    return result;
}

bool Robot::RunTraj(vector<vector<double>> posTraj, bool showAtten){
    vector<vector<int32_t>> cmdTraj = this->PoseTrajToCmdTraj(posTraj);
    int index = 0;
    if(showAtten){
        for(int i = 0; i < 16; i++){
            cout << endl;
        }
    }
    for(vector<int32_t> frame : cmdTraj){
        this->cable.MoveAllMotorCmd(frame);
        for(int i = 0; i < 6; i++)
            this->endEffectorPos[i] = posTraj[index][i];
        index++;
        
        if(showAtten){
            printf("\x1b[16A");
            this->PrintEEPos();
            cout << "Current Cable Motor Count: " << endl;
            for (int i = 0; i < this->cableMotorNum; i++){
                cout << "\tCable " << i << ": " << frame[i] << endl;
            }
        }
    }
    return true;
}

bool Robot::MoveToLinear(double dest[], int time, bool showAtten){
    return this->RunTraj(GenLinearTrajForCableMotor(this->endEffectorPos, dest, time, showAtten),showAtten);
};

bool Robot::MoveToParaBlend(double dest[], int time, bool showAtten){
    return this->RunTraj(GenParaBlendTrajForCableMotor(this->endEffectorPos, dest, time), showAtten);
}

bool Robot::MoveToParaBlend(double dest[], bool showAtten){
    float time = sqrt(pow(dest[0]-this->endEffectorPos[0],2)+pow(dest[1]-endEffectorPos[1],2)+pow(dest[2]-endEffectorPos[2],2))/this->velLmt*1000;
    return this->RunTraj(GenParaBlendTrajForCableMotor(this->endEffectorPos, dest, time), showAtten);
}

void Robot::SavePosToFile(string filename){
    
    //// Safe system shut down, safe last pos and emegency shut down
    // Saving last position before quiting programme
    cout << "Saving last position to file: "<< filename << endl;
    ofstream myfile;
    myfile.open (filename);
    for(int i = 0; i < 6; i++){
        myfile <<  this->endEffectorPos[i] << " ";
    }
    myfile << endl;

    for(int i = 0; i < this->railMotorNum; i++){
        myfile <<  this->railOffset[i] << " ";
    }
    myfile << endl;

    for(int i = 0; i < this->cableMotorNum; i++){
        myfile <<  this->cable.GetMotorPosMeasured(i) << " ";
    }

    vector<int> railMotorCmd = this->rail.GetMotorPosMeasured();
    for(int i = 0; i < this->railMotorNum; i++){
        myfile << railMotorCmd[i] << " ";
    }

    myfile << endl;

    
}