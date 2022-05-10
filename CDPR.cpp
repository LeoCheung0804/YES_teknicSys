#include "CDPR.h"
#include "json.hpp"
#include "Dependencies\eigen-3.3.7\Eigen\Dense"
#include <iostream>
#include <fstream>
#include <cmath>

using namespace Eigen;
using namespace std;
using json = nlohmann::json;

// Constructor, based on the requested model
CDPR::CDPR(){
    // Read json file into json object
    json model;
    try{
        ifstream file("model.json", ifstream::binary);
        file >> model;
        cout << "Imported model.json: " << model.value("ModelName", "NOT FOUND") << endl;
        
        // Initialize parameters
        this->nodeNum = model.value("tekMotorNum", 8); // Default value set in 2nd input
        this->absTorqueLmt = model.value("absTorqueLmt", 60);
        this->endEffOffset = model.value("endEffOffset", -0.28);
        this->targetTorque = model.value("targetTorque", -2.5);
        this->cmdScale = model.value("toMotorCmdScale", 509295);
        //this->railScale = model.value("railCmdScale", 38400000);
        this->pRadius = model.value("pulleyRadius", 0.045);
        // Interate through arrays
        for(int i=0; i<6; i++){
            this->home[i] = model["home"][i];
            this->limit[i] = (float) model["limit"][i];
        }
        for(int i=0; i<nodeNum; i++){
            frmOutUnitV[i] << 0, 0, model["pulleyZdir"][i];
            for(int j=0; j<3; j++){ // for each xyz coordinates
                this->frmOut[i][j] = model["frameAttachments"][i][j];
                this->endOut[i][j] = model["endEffectorAttachments"][i][j];
            }
        }
    }
    catch(const exception& e){
        cout << "Error in reading \"model.json\"" << endl;
        cout << e.what() << endl;
        isValidModel = false;
    }
}

bool CDPR::IsGood(){ return isValidModel; }
bool CDPR::CheckLimits(){
    for (int i = 0; i < 3; i++){
        if(limit[i*2]>in[i] || in[i]>limit[i*2+1]){ return false; }
    }
    return true;
}

void CDPR::PrintIn(){ // Print in[]
    cout << "IN: "<< in[0] << " " << in[1] << " " << in[2] << " " << in[3] << " " << in[4] << " " << in[5] << endl;
}

void CDPR::PrintOut(){ // Print out[]
    cout << "OUT: ";
    for(int i = 0; i < nodeNum; i++){ cout << out[i] << " "; }
    cout << endl;
}

void CDPR::PrintHome(){ // Print home[]
    cout << "HOME: "<< home[0] << " " << home[1] << " " << home[2] << " " << home[3] << " " << home[4] << " " << home[5] << endl;
}

void CDPR::PoseToLength(double pose[], double lengths[], double rail_offset){ // given an EE pose, calculate the EE to pulley cable lengths
    // local variables
    Vector3d orB[8]; // vector from frame 0 origin to end effector cable outlet
    Vector3d orA[8]; // vector from frame 0 origin to cable outlet point on rotating pulley
    Matrix3d oe_R; // rotation matrix from end effector to frame 0
    Matrix3d Ra, Rb, Rc;
    double a = pose[3], b = pose[4], c = pose[5];
    
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
    Vector3d endEfr(pose[0], pose[1], pose[2]); // Translation of end-effector
    for(int i = 0; i < nodeNum; i++){
        ///// Calculate orB[8] vectors /////
        orB[i] = oe_R*endOut[i] + endEfr;
                
        ///// Calculate cable outlet point on rotating pulley, ie point A /////
        Vector3d plNormal = frmOutUnitV[i].cross(orB[i] - frmOut[i]); // normal vector of the plane that the rotating pulley is in
        Vector3d VecC = plNormal.cross(frmOutUnitV[i]); // direction from fixed point towards pulley center
        Vector3d orC = VecC/VecC.norm()*pRadius + frmOut[i]; // vector from frame 0 origin to rotating pulley center
        double triR = pRadius / (orB[i] - orC).norm(); // triangle ratio??
        orA[i] = orC + triR*triR*(orB[i] - orC) - triR*sqrt(1 - triR*triR)*((plNormal/plNormal.norm()).cross(orB[i] - orC));
        
        ///// Calculate arc length /////
        Vector3d UVecCF = -VecC / VecC.norm(); // unit vector of CF
        Vector3d UVecCA = (orA[i]- orC) / (orA[i]- orC).norm(); // unit vector of CA
        double l_arc = pRadius * acos(UVecCF.dot(UVecCA));

        ///// Sum the total cable length /////
        lengths[i] = i < 4 ? l_arc + (orA[i] - orB[i]).norm() : l_arc + (orA[i] - orB[i]).norm() - rail_offset; // Need to subtract the rail offset for the bottom 4 motors 
    }
}

double CDPR::EEOffset(){ return endEffOffset; }
float CDPR::TargetTorque(){ return targetTorque; }
float CDPR::AbsTorqLmt(){ return absTorqueLmt; }
int CDPR::NodeNum(){ return nodeNum; }
int32_t CDPR::MotorScale(){ return cmdScale; }
// int32_t CDPR::RailScale(){ return railScale; }

int32_t CDPR::ToMotorCmd(int motorID, double length){
    if(motorID == -1) { return length * cmdScale; }
    return (length - this->offset[motorID]) * cmdScale;
}

/* void pose_to_length(double pose[], double lengths[], double rail_offset){
////////////////////////////////// Define the cable robot parameters here !! //////////////////////////////////
    const int CABLE_NUM = 8;
    const int direction[8] = {-1,-1,-1,-1,1,1,1,1}; // 1 for cable going over the pulley, -1 for going under pulley

    Vector3d frmOut[CABLE_NUM]; // coordinates of the attachment points on frame at the beginning
    frmOut[0] << -0.1181, 0.2896, -3.0638;
    frmOut[1] << 11.4851, -0.0752, -3.1883;
    frmOut[2] << 10.7566, 11.2062, -3.1766;
    frmOut[3] << 1.5580, 13.8996, -3.1681;
    frmOut[4] << -0.0228, -0.0254, 0.0474;
    frmOut[5] << 11.7866, -0.0308, 0.0203;
    frmOut[6] << 10.6577, 11.5044, 0.00408;
    frmOut[7] << 1.2524, 13.8546, 0.0554;
    // frmOut[4] << 0.00, -0.0325, -0.050 + rail_offset; //motors connected to linear rails, coordinates when rail offset is 0
    // frmOut[5] << 3.6647, -0.0325, -0.0523 + rail_offset;
    // frmOut[6] << 3.6760, 3.2815, -0.05332 + rail_offset;
    // frmOut[7] << 0.0145, 3.2815, -0.0529 + rail_offset;

    Vector3d frmOutUnitV[CABLE_NUM]; // unit vectors/directions of the fixed cable attachments on frame
    for(int i = 0; i < CABLE_NUM; i++){ frmOutUnitV[i] << 0, 0, direction[i]; }

    Vector3d endOut[CABLE_NUM]; // local coordinates of cable attachment points on end-effector, ie ^er_B
    endOut[0] << -0.221, -0.105, 0.103;
    endOut[1] << 0.221, -0.1075, 0.103;
    endOut[2] << 0.2215, 0.1075, 0.103;
    endOut[3] << -0.2215, 0.105, 0.103;
    endOut[4] << -0.220, -0.109, -0.103;
    endOut[5] << 0.220, -0.109, -0.103;
    endOut[6] << 0.224, 0.109, -0.103;
    endOut[7] << -0.224, 0.109, -0.103;

    const double pRadius = 0.045; // radius of rotating pulley on frame //0.025

////////////////////////////////// End of manual model defination !! //////////////////////////////////    
    // local variables
    Vector3d orB[CABLE_NUM]; // vector from frame 0 origin to end effector cable outlet
    Vector3d orA[CABLE_NUM]; // vector from frame 0 origin to cable outlet point on rotating pulley
    Matrix3d oe_R; // rotation matrix from end effector to frame 0
    Matrix3d Ra, Rb, Rc;
    double a = pose[3], b = pose[4], c = pose[5];
    
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
    Vector3d endEfr(pose[0], pose[1], pose[2]); // Translation of end-effector
    for(int i = 0; i < CABLE_NUM; i++){
        ///// Calculate orB[8] vectors /////
        orB[i] = oe_R*endOut[i] + endEfr;
                
        ///// Calculate cable outlet point on rotating pulley, ie point A /////
        Vector3d plNormal = frmOutUnitV[i].cross(orB[i] - frmOut[i]); // normal vector of the plane that the rotating pulley is in
        Vector3d VecC = plNormal.cross(frmOutUnitV[i]); // direction from fixed point towards pulley center
        Vector3d orC = VecC/VecC.norm()*pRadius + frmOut[i]; // vector from frame 0 origin to rotating pulley center
        double triR = pRadius / (orB[i] - orC).norm(); // triangle ratio??
        orA[i] = orC + triR*triR*(orB[i] - orC) - triR*sqrt(1 - triR*triR)*((plNormal/plNormal.norm()).cross(orB[i] - orC));
        
        ///// Calculate arc length /////
        Vector3d UVecCF = -VecC / VecC.norm(); // unit vector of CF
        Vector3d UVecCA = (orA[i]- orC) / (orA[i]- orC).norm(); // unit vector of CA
        double l_arc = pRadius * acos(UVecCF.dot(UVecCA));

        ///// Sum the total cable length /////
        lengths[i] = i < 4 ? l_arc + (orA[i] - orB[i]).norm() : l_arc + (orA[i] - orB[i]).norm() - rail_offset; // Need to subtract the rail offset for the bottom 4 motors 
    }
    lengths[CABLE_NUM] = lengths[CABLE_NUM+1] = lengths[CABLE_NUM+2] = lengths[CABLE_NUM+3] = rail_offset;
} */