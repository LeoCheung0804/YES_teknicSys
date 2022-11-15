#ifndef Trajectory_Generator_H
#define Trajectory_Generator_H

#include <vector>
using namespace std;

const int MILLIS_TO_NEXT_FRAME = 20;
vector<vector<double>> GenParaBlendForCableMotor(double start[6], double end[6], int time, bool showAttention);
#endif