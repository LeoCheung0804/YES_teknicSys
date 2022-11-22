#include "..\include\CableController.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <assert.h>
#include <Windows.h>
#include <conio.h>

CableController::CableController(bool isOnline){ this ->isOnline = isOnline; }

void CableController::Connect(int cableNumber, int brakeNumber, string brakePortName, bool useMotor, bool useBraker){
    this->isOnline = isOnline;
    this->cableNumber = cableNumber;
    this->brakeNumber = brakeNumber;
    this->motorNode = TeknicNode();
    this->brakeNode = ArduinoBLENode();
    this->isConnected = true;
    this->brakeOnFlags = new bool(brakeNumber);
    this->useMotor = useMotor;
    this->useBraker = useBraker;
    if(this->isOnline){
        if(useMotor){
            if(!this->motorNode.Connect(cableNumber)){
                cout << "Failed to connect cable motors. Exit programme." << endl;
                this->isConnected = false;
            }
            this->nodeList = motorNode.GetNodeList();
        }
        if(useBraker){
            if(!this->brakeNode.Connect(brakePortName)) { 
                cout << "Failed to connect cable motor brakes. Exit programme.\n"; 
                this->isConnected = false;
            };  
        }  
    }
    this->CloseAllBrake();
    cout << "Cable Controller Online." << endl;
}

void CableController::Disconnect(){
    this->CloseAllBrake();
    if(this->isOnline){
        if(this->useMotor)
            this->motorNode.Disconnect();
        if(this->useBraker)
            this->brakeNode.Disconnect();
    }
    cout << "Cable Controller Offline." << endl;
    this->isConnected = false;
}

bool CableController::IsConnected(){ return this->isConnected; }

void CableController::TightenCableByIndex(int index, float targetTrq){
    assert(index >= 0 && index <= cableNumber);
    cout << "Setting cable: " << index << " torque to: " << targetTrq << endl;
    this->OpenAllBrake();
    if(isOnline && this->useMotor){
        nodeList[index]->Motion.AccLimit = 200;
        // get current trq
        nodeList[index]->Motion.TrqCommanded.Refresh();
        float currentTrq = nodeList[index]->Motion.TrqCommanded.Value();
        // check current trq and set move direction
        if(currentTrq > targetTrq){ 
            nodeList[index]->Motion.MoveVelStart(-100); 
        }
        else if (currentTrq < targetTrq - 1.8){ 
            nodeList[index]->Motion.MoveVelStart(100);
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
    this->CloseAllBrake();
    cout << "Set cable " << index << " torque to: " << targetTrq << " finished." << endl;
    
}

void CableController::TightenAllCable(float targetTrq){
    cout << "Setting all cable torque to: " << targetTrq << endl;
    this->OpenAllBrake();
    if(isOnline && this->useMotor){
        bool moving = true;
        int index = 0;
        cout << "current torques: " << endl;
        for(INode* node : nodeList){
            node->Motion.AccLimit = 200;
            // get current trq
            node->Motion.TrqCommanded.Refresh();
            float currentTrq = node->Motion.TrqCommanded.Value();
            cout << "motor: " << index << ": " << currentTrq << "       " << endl;
            // check current trq and set move direction
            index++;
        }
        // -5 < trq < -3.5
        while(moving){
            printf("\x1b[9A");
            moving = false;
            cout << "current torques: " << endl;
            int index = 0;
            for(INode* node : nodeList){
                // get current trq
                node->Motion.TrqCommanded.Refresh();
                float currentTrq = node->Motion.TrqCommanded.Value();
                if(currentTrq > targetTrq){ 
                    node->Motion.MoveVelStart(-10); 
                    moving = true;
                }
                else if (currentTrq < targetTrq - 1.5){ 
                    node->Motion.MoveVelStart(10);
                    moving = true;
                }
                cout << "motor: " << index << ": " << currentTrq << "       " << endl;
                // check current trq
                if(currentTrq > targetTrq || currentTrq < targetTrq - 2)
                    moving = true; // continue loop 
                    index ++;
            }
            if(kbhit()){
                break;
            }
        }

        for(INode* node : nodeList){
            node->Motion.AccLimit = 40000;
            node->Motion.NodeStop(STOP_TYPE_ABRUPT);
        }
        
    }
    this->CloseAllBrake();
    cout << "Set all cable torque to: " << targetTrq << " finished." << endl;
}

void CableController::HomeAllMotors(){
    cout << "Moving all cable motors to zero position." << endl;
    this->OpenAllBrake();
    if(isOnline && this->useMotor){
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
    this->CloseAllBrake();
    cout << "Move all cable motors to zero position finished." << endl;
}

void CableController::MoveSingleMotorCmd(int index, int32_t cmd, bool absolute){
    assert(index >= 0 && index <= cableNumber);
    assert(!this->brakeOnFlags[index/2]);
    if(this->isOnline && this->useMotor){
        ofstream myfile;
        try{
            nodeList[index]->Motion.MovePosnStart(cmd, absolute, true); // absolute position?
            if (nodeList[index]->Status.Alerts.Value().isInAlert()) {
                myfile.open("log.txt", ios::app);
                myfile << "Alert from Motor [" << index << "]: "<< nodeList[index]->Status.Alerts.Value().bits <<"\n";
                myfile.close();
            }
        }
        catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
            cout << "\nERROR: Motor [" << index << "] command failed.\n";  
            printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            // quitType = 'e';
            tm *fn; time_t now = time(0); fn = localtime(&now);
            myfile.open("log.txt", ios::app);
            myfile << "\nERROR: Motor [" << index << "] command failed. " << fn->tm_hour << ":"<< fn->tm_min << ":" << fn->tm_sec <<"\n";
            myfile << "Caught error: addr="<< (int) theErr.TheAddr<<", err="<<hex<<theErr.ErrorCode <<"\nmsg="<<theErr.ErrorMsg<<"\n";
            myfile.close();
        }
    }else{
        Sleep(10);
    }
}

void CableController::MoveAllMotorCmd(vector<int32_t> cmdList, bool absolute){
    
    auto start = chrono::steady_clock::now();

    vector<thread> threadList;
    int index = 0;
    for(int32_t cmd : cmdList){
        threadList.push_back(thread(&CableController::MoveSingleMotorCmd, this, index, cmd, absolute));
        index++;
    }
    for(thread &thr : threadList){
        thr.join();
        index++;
    }
    
    auto end = chrono::steady_clock::now();
    
    double dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
    if(dif > 0) { Sleep(dif);}
}

void CableController::MoveAllMotorCmd(int cmd, bool absolute){
    vector<int32_t> cmdList;
    for(int i = 0; i < this->cableNumber; i++){ cmdList.push_back(cmd); }
    this->MoveAllMotorCmd(cmdList, absolute);
}

bool CableController::IsMoveFinished(){ return isMoveFinished; }

void CableController::CalibrationMotor(int index, int32_t currentCmdPos){
    assert(index >= 0 && index <= cableNumber);
    if(this->isOnline && this->useMotor){
        nodeList[index]->Motion.PosnMeasured.Refresh();
        nodeList[index]->Motion.AddToPosition(-nodeList[index]->Motion.PosnMeasured.Value() + currentCmdPos);
    }else{
        cout << "Offline mode, skip calibration" << endl;
    }
}

double CableController::GetMotorPosMeasured(int index){
    assert(index >= 0 && index <= cableNumber);
    if(this->isOnline && this->useMotor){
        nodeList[index]->Motion.PosnMeasured.Refresh();
        return (double) nodeList[index]->Motion.PosnMeasured.Value();
    }else{
        return 0;
    }
}

double CableController::GetMotorTorqueMeasured(int index){
    assert(index >= 0 && index <= cableNumber);
    if(this->isOnline && this->useMotor){
        nodeList[index]->Motion.TrqMeasured.Refresh();
        return (double) nodeList[index]->Motion.TrqMeasured.Value();
    }else{
        return 0;
    }
}

void CableController::OpenBrake(int index){
    assert(index >= 0 && index <= this->brakeNumber);
    this->sendStr = "(0:0)   ";
    this->sendStr[3] = ('0' + index);
    // cout << "Sending Command: " << this->sendStr << " to cable brakes" << endl;
    if(this->isOnline && this->useBraker)
        brakeNode.Send(this->sendStr);
    this->brakeOnFlags[index] = false;
    cout << "Cable Brake " << index << " Opened." << endl;
}

void CableController::CloseBrake(int index){
    assert(index >= 0 && index <= this->brakeNumber);
    this->sendStr = "(1:0)   ";
    this->sendStr[3] = ('0' + index);
    // cout << "Sending Command: " << this->sendStr << " to cable brakes" << endl;
    if(this->isOnline && this->useBraker)
        brakeNode.Send(this->sendStr);
    this->brakeOnFlags[index] = true;
    cout << "Cable Brake " << index << " Closed." << endl;
}

void CableController::OpenAllBrake(){
    for(int i = 0; i < this->brakeNumber; i++){
            this->OpenBrake(i);
        }
}

void CableController::CloseAllBrake(){
    for(int i = 0; i < this->brakeNumber; i++){
            this->CloseBrake(i);
        }
}
