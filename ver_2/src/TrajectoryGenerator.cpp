#define _USE_MATH_DEFINES
#include "..\include\TrajectoryGenerator.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>

vector<vector<double>> GenLinearTrajForCableMotor(double start[], double end[], int time, bool printLog){
    if(printLog){
        cout << "Generating linear trajectory from " ;
        cout << start[0] << ", " << start[1] << ", " << start[2] << ", " << start[3] << ", " << start[4] << ", " << start[5];
        cout << " to ";
        cout << end[0] << ", " << end[1] << ", " << end[2] << ", " << end[3] << ", " << end[4] << ", " << end[5];
        cout << " in " << time << " ms" << endl;
    }
    vector<vector<double>> result;
    double step[6]{0};
    double t = 0;
    for(int i = 0; i < 6; i++){
        step[i] = (end[i] - start[i]) / time;
    }
    while (t <= time){        
        // PARABOLIC BLEND equation, per time step pose
        vector<double> frame;
        for (int i = 0; i < 6; i++){
            frame.push_back(start[i] + step[i] * t);
        }
        result.push_back(frame);
        // next frame
        t += MILLIS_TO_NEXT_FRAME;
    }
    if(printLog){
        cout << "Generation finished." << endl;
        cout << "total frame number: " << result.size() << endl;
    }
    // cout << "total used space: " << sizeof(vector<vector<float>>) + result.capacity() * sizeof(vector<vector<float>>) << endl;

    return result;
}

vector<vector<double>> GenParaBlendTrajForCableMotor(double start[], double end[], int time, bool printLog){
    vector<vector<double>> result;
    if(time <= 200){ 
        cout << "Time too short, traj won't run" << endl;
        return result; 
    } // Don't run traj for incorrect timing 

    if(printLog){
        cout << "Generating parabolic blend trajectory." << endl;
        cout << "From: " << endl;
        cout << "\t" << start[0] << ", " << start[1] << ", " << start[2] << ", " << start[3] << ", " << start[4] << ", " << start[5] << endl;
        cout << " To: " << endl;
        cout << "\t" << end[0] << ", " << end[1] << ", " << end[2] << ", " << end[3] << ", " << end[4] << ", " << end[5] << endl;
        cout << " In " << time << " ms" << endl;
    }
    // cout << sizeof(start) << " " << sizeof(end) << endl;
    float vMax[6] = {.6, .6, .6, 0.8, 0.8, 0.8}; // m/s, define the maximum velocity for each DoF
    float aMax[6] = {80, 80, 80, 10, 10, 10}; // m/s^2, define the maximum acceleration for each DoF
    static double a[6], b[6], c[6], d[6], e[6], f[6], g[6], tb[6]; // trajectory coefficients
    static double sQ[6], Q[6], o[6];
    double unitV = sqrt(pow(end[0]-start[0],2)+pow(end[1]-start[1],2)+pow(end[2]-start[2],2)); // the root to divide by to get unit vector
    

    // Solve parabolic blend coefficients for each DoF
    for(int i = 0; i < 6; i++){
        sQ[i] = start[i]; // start point, from current position
        Q[i] = end[i]; // end point, from goal position
        vMax[i] /= 1000; // change the velocity unit to meter per ms
        aMax[i] /= 1000000; // change the unit to meter per ms square
        tb[i] = i<3? time-unitV/vMax[i] : time-abs(Q[i]-sQ[i])/vMax[i];
        if(tb[i] < 0) {
            if(printLog)
                cout << "WARNING: Intended trajectory exceeds velocity limit in DoF "<< i << ".\n";
            return result;
        }
        else if (tb[i] > time / 2){
            if(printLog)
                cout << "ATTENTION: Trajectory for DoF " << i << " will be in cubic form.\n";
            tb[i] = time / 2;
            vMax[i] = 2 * (Q[i] - sQ[i]) / time;
        }
        else if(i<3){ vMax[i] = vMax[i] * (Q[i] - sQ[i]) / unitV; } // vMax in x,y,z accordingly
        else if(Q[i]<sQ[i]){ vMax[i] *= -1; } //Fix velocity direction for rotation
        o[i] = vMax[i] / 2 / tb[i]; // times 2 to get acceleration
        if(abs(o[i]*2) > aMax[i]){
            if(printLog)
                cout << "WARNING: Intended trajectory acceleration <" << abs(o[i]*2) << "> exceeds limit in DoF "<< i << ".\n";
            return result;
        }
        
        // solve coefficients of equations for parabolic
        a[i] = sQ[i];
        b[i] = o[i];

        c[i] = sQ[i] - vMax[i] * tb[i] / 2;
        d[i] = vMax[i];

        e[i] = Q[i] - o[i] * time * time;
        f[i] = 2 * o[i] * time;
        g[i] = -o[i];
    }
    
    double t = 0;
    while (t <= time){        
        // PARABOLIC BLEND equation, per time step pose
        vector<double> frame;
        for (int j = 0; j < 6; j++){
            if (t <= tb[j]){
                frame.push_back(a[j] + b[j] * t * t);
            }
            else if(t <= time-tb[j]){
                frame.push_back(c[j] + d[j] * t);
            }
            else{
                frame.push_back(e[j] + f[j] * t + g[j] * t * t);
            }
        }
        result.push_back(frame);
        // next frame
        t += MILLIS_TO_NEXT_FRAME;
    }
    if(printLog){
        cout << "Generation finished." << endl;
        cout << "total frame number: " << result.size() << endl;
    }
    // cout << "total used space: " << sizeof(vector<vector<float>>) + result.capacity() * sizeof(vector<vector<float>>) << endl;

    return result;
}

vector<vector<double>> ReadBrickPosFile(string filename, float tempA, float tempD){
    ifstream file(filename);
    vector<vector<double>> result;
    vector<double> row;
    string line;
    string word;
    string temp;

    if(file.is_open()){
        while (getline(file, line)){
            row.clear();
            stringstream s(line);
            while (s >> word){
                row.push_back(stod(word)); // convert string to double stod()
            }
            // Convert angles
            if(row[4]<0){ row[4] += 180; } // convert -ve degs to 180
            // if(row[4]>180){ row[4] -= 180; } // convert 360 degs to 180
            // Calculate and add x, y offsets due to rotation mis-alignment
            row[0] -= tempD*sin((tempA + row[4])*M_PI/180);
            row[1] += tempD*cos((tempA + row[4])*M_PI/180);
            result.push_back(row);
        }
        cout << "Completed reading brick position input file" << endl;
        return result;
    }else{ 
        cout << "Failed to read input file. Please check the brick trajectory file: " << filename << endl; 
        return result;
    };
}

vector<vector<double>> ReadTrajFile(string filename){
    
    // Read raw points from extern.csv
    ifstream file(filename);
    vector<vector<double>> result;
    vector<double> row;
    string line;
    string word;
    string temp;

    if(file.is_open()){
        while (getline(file, line)){
            row.clear();
            stringstream s(line);
            while (s >> word){
                row.push_back(stod(word)); // convert string to double stod()
            }
            result.push_back(row);
        }
        cout << "Completed reading external traj step points input file" << endl;
        return result;
    }
    else{ 
        cout << "Failed to read input file. Please check file: " << filename << endl; 
        return result;
    }
}

vector<vector<double>> ReadPointFile(string filename){
    
    // Read raw points from extern.csv
    ifstream file(filename);
    vector<vector<double>> result;
    vector<double> row;
    string line;
    string word;
    string temp;

    if(file.is_open()){
        while (getline(file, line)){
            row.clear();
            stringstream s(line);
            while (s >> word){
                row.push_back(stod(word)); // convert string to double stod()
            }
            result.push_back(row);
        }
        cout << "Completed reading point path file" << endl;
        return result;
    }
    else{ 
        cout << "Failed to read input file. Please check file: " << filename << endl; 
        return result;
    }
}
