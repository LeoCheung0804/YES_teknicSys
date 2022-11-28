#pragma comment(lib, "User32.lib")
#include "../include/TeknicNode.h"
#include "../include/Logger.h"
#include <Windows.h>
#include <iostream>
#include <conio.h>
#include <thread>

using namespace std;

TeknicNode motorNode;
INode* node;
void main(){
    Logger logger;
    logger.NewFile("log\\trq.log");
    motorNode.Connect(1);
    node = motorNode.GetNode(0);
    cout << endl;
    // while(true){
    //     // printf("\x1b[1A");
    //     cout << node->Motion.TrqMeasured.Value() << endl;
    //     logger.LogInfo(to_string(node->Motion.TrqMeasured.Value()));
    //     if(GetKeyState('Q') & 0x8000){
    //         break;
    //     }else if(GetKeyState('O') & 0x8000){
    //         node->Motion.MoveVelStart(10);
    //     }else if(GetKeyState('P') & 0x8000){
    //         node->Motion.MoveVelStart(-10);
    //     }else{
    //         node->Motion.NodeStop(STOP_TYPE_ABRUPT);
    //     }
    // }
    system("pause");
    node->Motion.MoveVelStart(40);
    while(true){
        cout << node->Motion.TrqMeasured.Value() << endl;
        if(kbhit()){
        node->Motion.NodeStop(STOP_TYPE_ABRUPT);
            break;
        }
    }
    node->Motion.NodeStop(STOP_TYPE_ABRUPT);
    system("pause");
    float currentTrq = node->Motion.TrqMeasured.Value();
    node->Motion.MoveVelStart(-20);
    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();
    double dif = 50 - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
    while(currentTrq>-12){
        end = chrono::steady_clock::now();
        dif = 50 - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
        if(dif < 0){
            start = chrono::steady_clock::now();
            currentTrq = node->Motion.TrqMeasured.Value();
            cout << currentTrq << endl;
            logger.LogInfo(to_string(currentTrq));
        }
        if(kbhit()){
            node->Motion.NodeStop(STOP_TYPE_ABRUPT);
                break;
        }
    }
    node->Motion.NodeStop(STOP_TYPE_ABRUPT);
    
    while(true){;
        end = chrono::steady_clock::now();
        dif = 50 - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
        if(dif < 0){
            start = chrono::steady_clock::now();
            currentTrq = node->Motion.TrqMeasured.Value();
            cout << currentTrq << endl;
            logger.LogInfo(to_string(currentTrq));
        }
        if(kbhit()){
            break;
        }
    }

    system("pause");


    motorNode.Disconnect();
    logger.CloseFile();
}