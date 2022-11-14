#ifndef Robot_H
#define Robot_H

#include "..\Dependencies\eigen-3.3.7\Eigen\Dense"
#include <string>
using namespace Eigen;
using namespace std;

// CDPR with specific robot parameters
class Robot{
private:
    bool isValidModel{true};
    int cableMotorNum{8}; // !!!!! IMPORTANT !!!!! Put in the number of motors for cables before compiling the programme
    int railMotorNum{4}; // Typical 4 linear rails
    double endEffToGroundOffset = -0.125; // offset from endeffector to ground, in meters // YES -0.280
    double cableMotorScale = 509295.8179; // 6400 encoder count per revoltion, 25 times gearbox, 50mm spool radias. ie 6400*25/(2*pi*0.05) 
    double railMotorScale = 38400000; // 6400 encoder count per revoltion, 30 times gearbox, linear rail pitch 5mm. ie 6400*30/0.005 
    double pulleyRadius = 0.045; // radius of rotating pulley on frame
    float targetTrq = -2.5; // in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
    float absTrqLmt = 60.0; // Absolute torque limit, to be checked before commanding 8 motors to move simutaneously in length command
    float endEffectorPosLimit[6]{-0.1, 11.5, -0.02, 13.8, -3.3, 0.3}; // Rough boundaries of the cable robot
    static const double defaultRailOffset[4]; // default rail Position
    Vector3d frmOut[8]; // coordinates of the attachment points on frame at the beginning
    Vector3d endOut[8]; // local coordinates of cable attachment points on end-effector, ie ^er_B
    Vector3d frmOutUnitV[8]; // unit vectors/directions of the fixed cable attachments on frame
    string gripperCommPort; // Gripper Communicate Port 
    string railBreakCommPort; // Rail Break Communicate Port 
    
public:
    Robot();
    Robot(string robotConfigPath);

    /// @brief Check whether the robot is valid, ususally used after load the robot config.
    /// @return Bool
    bool IsValid();
    /// @brief Check whether the end effector position is in the limits or not.
    /// @return Bool.
    bool CheckLimits();
    /// @brief Pint current end effector pos
    void PrintEEPos();
    /// @brief Print current rail offset
    void PrintRailOffset();
    /// @brief Print current cable length 
    void PrintCableLength();
    /// @brief Print home pos
    void PrintHomePos();
    /// @brief Print robot config file
    void PrintRobotConfig();
    /// @brief Read robot model config from file.
    /// @param filename string, Config file name.
    void UpdateModelFromFile(std::string filename);
    /// @brief Give end effector pos and rail offset, get cable length and update the lengths attribute. 
    /// @param eePos double array, Input end effector pos
    /// @param railOffset double array, Input rail offset. Leave empty will use default offset.
    /// @param lengths double array, Output cable length
    void EEPoseToCableLength(double eePos[], double railOffset[], double lengths[]);
    /// @brief Get end effector offset from end effector to ground, in meter.
    /// @return Double, Effector offset from end effector to ground, in meter.
    double GetEEToGroundOffset();
    /// @brief Get target torque.
    /// @return float, Target torque.
    float GetTargetTrq();
    /// @brief Absolute torque limit
    /// @return float, Absolute torque limit
    float GetAbsTrqLmt();
    /// @brief Get cable motors number.
    /// @return int, number of cable motor
    int GetCableMotorNum();
    /// @brief Get rail motors number.
    /// @return int, number of rail motor
    int GetRailMotorNum();
    /// @brief Get Gripper Comm Port 
    /// @return string
    string GetGripperCommPort();
    /// @brief Get Rail Break Comm Port
    /// @return string 
    string GetRailBreakCommPort();
    /// @brief Cable motor scale
    /// @return Cable motor scale
    int32_t GetCableMotorScale();
    /// @brief Rail motor scale
    /// @return Rail motor scale
    int32_t GetRailMotorScale();
    /// @brief Convert cable length to motor command
    /// @param motorID int, motor ID
    /// @param length double, cable length
    /// @return int32_t motor command
    int32_t CableMotorLengthToCmd(int motorID, double length);
    /// @brief Convert rail height to motor command
    /// @param motorID int, motor ID
    /// @param length double, cable length
    /// @return int32_t motor command
    int32_t RailMotorOffsetToCmd(int motorID, double length);
    /// @brief Convert cable length to absulote motor command 
    /// @param length double, cable length
    /// @return int32_t motor command
    int32_t CableMotorLengthToCmdAbsulote(double length);
    double rotationalAngleOffset{0}; // rotational angel offset
    double rotationalDistanceOffset{0}; // rotational distance offset
    
    double homePos[6]{}; // home posisiton
    double endEffectorPos[6]{}; // end-effector task-space position {x,y,z, alpha, beta, gama}, in meter and radian
    double cableLength[8]{}; // cable lengths in meter //12 assume there are 8 motors + 4 linear rails
    double railOffset[4]{}; // individual heights or positions of the rails
    double robotOffset[12]{}; //12 //L0, from "zero position", will be updated by "set home" command
    double brickPickUpPos[6]{}; // position at 5th pole for brick pick up with possible z-rotaion
};

#endif