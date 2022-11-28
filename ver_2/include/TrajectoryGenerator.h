#ifndef Trajectory_Generator_H
#define Trajectory_Generator_H

#include <vector>
#include <string>
using namespace std;

const int MILLIS_TO_NEXT_FRAME = 50;
vector<vector<double>> GenLinearTrajForCableMotor(double start[], double end[], int time, bool printLog=true);
vector<vector<double>> GenParaBlendTrajForCableMotor(double start[6], double end[6], int time, bool printLog=true);
vector<vector<double>> ReadBrickPosFile(string filename, float tempD, float tempA);
vector<vector<double>> ReadTrajFile(string filename);
vector<vector<double>> ReadPointFile(string filename);
#endif