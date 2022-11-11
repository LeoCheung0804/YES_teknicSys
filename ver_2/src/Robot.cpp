#include "..\include\Robot.h"
#include "..\tools\json.hpp"
#include "..\Dependencies\eigen-3.3.7\Eigen\Dense"
#include <iostream>
#include <fstream>
#include <cmath>

using namespace Eigen;
using namespace std;
using json = nlohmann::json;

// Constructor, based on the requested model
Robot::Robot(string robotConfigPath){
    this->UpdateModelFromFile(robotConfigPath);
}

const double Robot::defaultRailOffset[4] = {0, 0, 0, 0}; // default all zeros

bool Robot::IsValid(){ return isValidModel; }
bool Robot::CheckLimits(){
    for (int i = 0; i < 3; i++){
        if(endEffectorPosLimit[i*2]>endEffectorPos[i] || endEffectorPos[i]>endEffectorPosLimit[i*2+1]){ return false; }
    }
    return true;
}

void Robot::PrintEEPos(){ 
    cout << "End Effector Pos: ";
    for(int i = 0; i < 6; i++){ cout << endEffectorPos[0] << " "; }
    cout << endl;
}

void Robot::PrintRailOffset(){ // Print railOffset[]
    cout << "Rail Offset: "<< railOffset[0] << " " << railOffset[1] << " " << railOffset[2] << " " << railOffset[3] << endl;
}

void Robot::PrintCableLength(){ 
    cout << "Cable Length: ";
    for(int i = 0; i < cableMotorNum; i++){ cout << cableLength[i] << " "; }
    cout << endl;
    cout << "Rail Offset: ";
    for(int i = cableMotorNum; i < railMotorNum + cableMotorNum; i++){ cout << cableLength[i] << " "; }
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
        string gripperCommPort = model.value("gripperCommPort", "\\\\.\\COM26");
        string railBreakCommPort = model.value("railBreakCommPort", "\\\\.\\COM26");
        this->gripperCommPort = &gripperCommPort[0];
        this->railBreakCommPort = &railBreakCommPort[0];
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
        isValidModel = true;
        PrintRobotConfig();
    }
    catch(const exception& e){
        cout << "Error in reading " << filename << endl;
        cout << e.what() << endl;
        isValidModel = false;
    }
}

void Robot::PrintRobotConfig(){
    
        string posLabel[] = {"x", "y", "z", "yaw", "pitch", "roll"};
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

        cout << "Home Pos: " << endl;
        for (int i = 0; i < 6; i++){
            cout << "\t" << posLabel[i] << ": " << this->homePos[i] << endl;
        }
        cout << "End Effector Pos Limit: " << endl;
        for (int i = 0; i < 6; i++){
            cout << "\t" << posLabel[i] << ": " << this->endEffectorPosLimit[i] << endl;
        }
        cout << "Brick Pick Up Pos: " << endl;
        for (int i = 0; i < 6; i++){
            cout << "\t" << posLabel[i] << ": " << this->brickPickUpPos[i] << endl;
        }
}

void Robot::EEPoseToCableLength(double eePos[], double railOffset[], double lengths[]){ // given an EE pose, calculate the EE to pulley cable lengths
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
        if(i < 4){ zOffset << 0, 0, railOffset[i]; }
        Vector3d orC = VecC/VecC.norm()*pulleyRadius + frmOut[i] + zOffset; // vector from frame 0 origin to rotating pulley center
        double triR = pulleyRadius / (orB[i] - orC).norm(); // triangle ratio??
        orA[i] = orC + triR*triR*(orB[i] - orC) - triR*sqrt(1 - triR*triR)*((plNormal/plNormal.norm()).cross(orB[i] - orC));
        
        ///// Calculate arc length /////
        Vector3d UVecCF = -VecC / VecC.norm(); // unit vector of CF
        Vector3d UVecCA = (orA[i]- orC) / (orA[i]- orC).norm(); // unit vector of CA
        double l_arc = pulleyRadius * acos(UVecCF.dot(UVecCA));

        ///// Sum the total cable length /////
        lengths[i] = i < 4 ? l_arc + (orA[i] - orB[i]).norm() - railOffset[i] : l_arc + (orA[i] - orB[i]).norm(); // Need to subtract the rail offset for the bottom 4 motors 
    }
    //copy(railOffset, railOffset+4, lengths+cableMotorNum); // assign array elements after motor numbers as rail lengths // rail_offset default size of 4
}

double Robot::GetEEToGroundOffset(){ return endEffToGroundOffset; }
float Robot::GetTargetTrq(){ return targetTrq; }
float Robot::GetAbsTrqLmt(){ return absTrqLmt; }
int Robot::GetCableMotorNum(){ return cableMotorNum; }
int Robot::GetRailMotorNum(){ return railMotorNum; }
char* Robot::GetGripperCommPort(){ return gripperCommPort; }
char* Robot::GetRailBreakCommPort(){ return railBreakCommPort; }
int32_t Robot::GetCableMotorScale(){ return cableMotorScale; }
int32_t Robot::GetRailMotorScale(){ return railMotorScale; }

int32_t Robot::CableMotorLengthToCmdAbsulote(double length){
    return length * cableMotorScale;
}
int32_t Robot::CableMotorLengthToCmd(int motorID, double length){
    return (length - this->robotOffset[motorID]) * cableMotorScale;
}
int32_t Robot::RailMotorOffsetToCmd(int motorID, double length){
    return (length - this->robotOffset[motorID]) * railMotorScale; 
}
