#include "..\include\CableController.h"
#include <iostream>
#include <thread>

CableController::CableController(){};
CableController::CableController(int cableNumber, bool isOnline){
    this->isOnline = isOnline;
    this->cableNumber = cableNumber;
    node = TeknicNode(cableNumber);
    if(isOnline && !node.Connect()) { cout << "Failed to connect cable motor motors. Exit programme.\n"; exit(-1); };
    nodeList = node.GetNodeList();
}
void CableController::TightenCableByIndex(int index, float targetTrq){
    cout << "Setting cable: " << index << " torque to: " << targetTrq << endl;
    if(isOnline){
        nodeList[index]->Motion.AccLimit = 200;
        // get current trq
        nodeList[index]->Motion.TrqCommanded.Refresh();
        float currentTrq = nodeList[index]->Motion.TrqCommanded.Value();
        // check current trq and set move direction
        if(currentTrq > targetTrq){ 
            nodeList[index]->Motion.MoveVelStart(-10); 
        }
        else if (currentTrq < targetTrq - 1.8){ 
            nodeList[index]->Motion.MoveVelStart(10);
        }
        // move
        while(currentTrq > targetTrq || currentTrq < targetTrq - 1.8){
            nodeList[index]->Motion.TrqCommanded.Refresh();
            currentTrq = nodeList[index]->Motion.TrqCommanded.Value();
            cout <<"Cable " << index << " current torque: " << currentTrq << endl;
            // need to turn the print interval.
        }
        // stop
        nodeList[index]->Motion.NodeStop(STOP_TYPE_ABRUPT);
        nodeList[index]->Motion.AccLimit = 40000;
    }
    cout << "Set cable " << index << " torque to: " << targetTrq << " finished." << endl;
}
void CableController::TightenAllCable(float targetTrq){
    cout << "Setting all cable torque to: " << targetTrq << endl;
    if(isOnline){
        bool moving = false;
        for(INode* node : nodeList){
            node->Motion.AccLimit = 200;
            // get current trq
            node->Motion.TrqCommanded.Refresh();
            float currentTrq = node->Motion.TrqCommanded.Value();
            // check current trq and set move direction
            if(currentTrq > targetTrq){ 
                node->Motion.MoveVelStart(-10); 
                moving = true;
            }
            else if (currentTrq < targetTrq - 1.8){ 
                node->Motion.MoveVelStart(10);
                moving = true;
            }
        }
        while(moving){
            moving = false;
            cout << "current torques: ";
            for(INode* node : nodeList){
                // get current trq
                node->Motion.TrqCommanded.Refresh();
                float currentTrq = node->Motion.TrqCommanded.Value();
                cout << currentTrq << " ";
                // check current trq
                if(currentTrq > targetTrq || currentTrq < targetTrq - 1.8)
                    moving = true; // continue loop 
                else{
                    node->Motion.NodeStop(STOP_TYPE_ABRUPT);
                    node->Motion.AccLimit = 40000;
                }
                cout << endl;
            }
        }
    }
    cout << "Set all cable torque to: " << targetTrq << " finished." << endl;
}
void CableController::HomeAllCableMotors(){
    cout << "Moving all cable motors to zero position." << endl;
    if(isOnline){
        bool moving = false;
        for(INode* node : nodeList){
            node->Motion.MovePosnStart(0, true);
        }
        while(moving) {
            moving = false;
            for (INode* node : nodeList) {
                if(!node->Motion.MoveIsDone()) 
                    moving = true;
            }
        }
    }
    cout << "Move all cable motors to zero position finished." << endl;
}
void CableController::MoveToLength(float cableLength[]){
    
}
bool CableController::IsMoveFinished(){ return isMoveFinished; }


