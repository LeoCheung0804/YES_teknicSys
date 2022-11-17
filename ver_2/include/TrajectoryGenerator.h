#ifndef Trajectory_Generator_H
#define Trajectory_Generator_H

#include <vector>
#include <string>
using namespace std;

const int MILLIS_TO_NEXT_FRAME = 20;
vector<vector<double>> GenParaBlendTrajForCableMotor(double start[6], double end[6], int time, bool showAttention);
vector<vector<double>> ReadBrickPosFile(string filename, float tempD, float tempA);
#endif