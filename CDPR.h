#ifndef CDPR_H
#define CDPR_H

#include "Dependencies\eigen-3.3.7\Eigen\Dense"
using namespace Eigen;

// CDPR with specific robot parameters
class CDPR{
private:
    bool isValidModel{true};
    int nodeNum{8}; // !!!!! IMPORTANT !!!!! Put in the number of motors before compiling the programme
    int railNum{4}; // Typical 4 linear rails
    double endEffOffset = -0.125; // meters, offset from endeffector to ground // YES -0.280
    double cmdScale = 509295.8179; // 6400 encoder count per revoltion, 25 times gearbox, 50mm spool radias. ie 6400*25/(2*pi*0.05) 
    double railScale = 38400000; // 6400 encoder count per revoltion, 30 times gearbox, linear rail pitch 5mm. ie 6400*30/0.005 
    double pRadius = 0.045; // radius of rotating pulley on frame
    float targetTorque = -2.5; // in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
    float absTorqueLmt = 60.0; // Absolute torque limit, to be checked before commanding 8 motors to move simutaneously in length command
    float limit[6]{-0.1, 11.5, -0.02, 13.8, -3.3, 0.3}; // Rough boundaries of the cable robot
    static const double defaultRail[4]; // default rail offsets
    Vector3d frmOut[8]; // coordinates of the attachment points on frame at the beginning
    Vector3d endOut[8]; // local coordinates of cable attachment points on end-effector, ie ^er_B
    Vector3d frmOutUnitV[8]; // unit vectors/directions of the fixed cable attachments on frame
    
public:
    CDPR();

    bool IsGood();
    bool CheckLimits();
    void PrintIn();
    void PrintOut(bool showRail = false);
    void PrintRail();
    void PrintHome();
    void UpdateModel();
    void PoseToLength(double pose[], double lengths[], const double rail_offset[4] = defaultRail);
    double EEOffset();
    float TargetTorque();
    float AbsTorqLmt();
    int NodeNum();
    int RailNum();
    int32_t MotorScale();
    int32_t RailScale();
    int32_t ToMotorCmd(int motorID, double length);
    
    double home[6]{}; // home posisiton
    double in[6]{}; // end-effector task-space position {x,y,z, alpha, beta, gama}, in meter and radian
    double out[12]{}; // cable lengths in meter //12 assume there are 8 motors + 4 linear rails
    double offset[12]{}; //12 //L0, from "zero position", will be updated by "set home" command
    double railOffset[4]{}; // individual heights or positions of the rails
    double brickPickUp[6]{}; // position at 5th pole for brick pick up with possible z-rotaion
};

#endif