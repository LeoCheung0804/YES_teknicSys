#include "..\include\CableController.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <assert.h>
#include <Windows.h>
#include <conio.h>
#include <queue>

CableController::CableController(bool isOnline, bool useCable){ 
    this->motorNode = TeknicNode(isOnline);
    this->useCable = useCable; 
}

void CableController::Connect(int cableNumber){
    if(!this->useCable) return;
    this->cableNumber = cableNumber;
    this->logger.OpenFile("log\\trq.log");
    if(!this->motorNode.Connect(cableNumber)){
        cout << "Failed to connect cable motors. Exit programme." << endl;
        this->isConnected = false;
    }
    this->nodeList = motorNode.GetNodeList();
    cout << "Cable Controller Online." << endl;
    this->isConnected = true;
}

void CableController::Disconnect(){
    if(!this->useCable) return;
    this->logger.CloseFile();
    this->motorNode.Disconnect();
    cout << "Cable Controller Offline." << endl;
    this->isConnected = false;
}

bool CableController::IsConnected(){ return this->isConnected; }

void CableController::SetCableTrqByIndex(int index, float targetTrq, float tolerance){
    if(!this->useCable) return;
    cout << "Setting all cable torque to: " << targetTrq << endl;
        cout << "current torques: " << endl;
        motorNode.SetAccLmt(index, 200);
        // get current trq
        float currentTrq = motorNode.GetTrqMeasured(index);
        cout << "motor: " << index << ": " << currentTrq << "       " << endl;
        
        if(currentTrq > targetTrq){ 
            motorNode.SetVel(index, -10); 
            currentTrq = motorNode.GetTrqMeasured(index);
            while(currentTrq > targetTrq - tolerance){
                cout << "current torques: " << endl;
                // get current trq
                currentTrq = motorNode.GetTrqMeasured(index);
                cout << "motor: " << index << ": " << currentTrq << "       " << endl;
                if(kbhit()){
                    motorNode.StopNode(index);
                    break;
                }
            }
        }
        motorNode.StopNode(index);
        motorNode.SetAccLmt(index, 40000);
    cout << "Set cable " << index << " torque to: " << targetTrq << " finished." << endl;
    
}

void CableController::SetCableTrq(float targetTrq, float tolerance){
    if(!this->useCable) return;
    cout << "Setting all cable torque to: " << targetTrq << endl;
        bool* stoppFlag = new bool(this->cableNumber);
        bool moving = true;
        cout << "current torques: " << endl;
        float currentTrq = 0;
        for(int index = 0; index < cableNumber; index++){
            motorNode.SetAccLmt(index, 200);
            // get current trq
            currentTrq = motorNode.GetTrqMeasured(index);
            cout << "motor: " << index << ": " << currentTrq << "       " << endl;
            // check current trq and set move direction
            if(currentTrq > targetTrq - tolerance){ 
                motorNode.SetVel(index, -10); 
                stoppFlag[index] = false;
            }
        }
        // -5 < trq < -3.5
        string log = "";
        auto start = chrono::steady_clock::now();
        auto end = chrono::steady_clock::now();
        double dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
        
        while(moving){
            end = chrono::steady_clock::now();
            dif = this->MILLIS_TO_NEXT_FRAME - chrono::duration_cast<chrono::milliseconds>(end-start).count() - 1;
            if(dif < 0){
                start = chrono::steady_clock::now();
                printf("\x1b[9A");
                moving = false;
                cout << "current torques: " << endl;
                log = "";
                
                for(int index = 0; index < cableNumber; index++){
                    // get current trq
                    currentTrq = motorNode.GetTrqMeasured(index);
                    if (currentTrq < targetTrq - tolerance || stoppFlag[index]){ 
                        motorNode.StopNode(index); 
                        stoppFlag[index] = true;
                    }else{
                        moving = true;
                    }
                    cout << "motor: " << index << ": " << currentTrq << "       " << endl;
                    log +=  to_string(currentTrq) + ",";
                }
                logger.LogInfo(log);
            }
            if(kbhit()){
                moving = false;
                break;
            }
        }
        motorNode.StopAll();
        for(int index = 0; index < this->cableNumber; index++){
            motorNode.SetAccLmt(index, 40000);
        }
        
    cout << "Set all cable torque to: " << targetTrq << " finished." << endl;
}

void CableController::SetTrqLmt(float lmt){
    this->absTrqLmt = lmt;
}

void CableController::ClearAlert(){
    if(!this->useCable) return;
    motorNode.ClearAlert();
    cout << "Alert Cleared!" << endl;    
}

void CableController::StopAllMotor(){
    if(!this->useCable) return;
    motorNode.StopAll();
    cout << "All Node Stopped!" << endl;    
}

bool CableController::MoveSingleMotorCmd(int index, int32_t cmd, bool absolute){
    if(absolute){
        cout << "Move Single Motor Cmd: " << index << " to " << cmd << endl;
    }else{
        cout << "Move Single Motor Cmd: " << index << " by " << cmd << endl;
    }
    if(!this->useCable) return true;
    assert(index >= 0 && index <= cableNumber);
        ofstream myfile;
        try{
            motorNode.MoveToPosn(index, cmd, absolute);// absolute position?
            
            if (motorNode.NodeIsInAlert(index)) {
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
            this->eStop = true;
            return false;
        }
    return true;
}

bool CableController::MoveAllMotorCmd(vector<int32_t> cmdList, bool absolute){
    if(!this->useCable) return true;
    this->eStop = false;
    vector<thread> threadList;
    int index = 0;
    // thread t1(&CableController::CheckTrq, this);
    for(int32_t cmd : cmdList){
        threadList.push_back(thread(&CableController::MoveSingleMotorCmd, this, index, cmd, absolute));
        index++;
    }
    for(thread &thr : threadList){
        thr.join();
    }
    if(this->eStop)
        return false;
    return true;
}

bool CableController::MoveAllMotorCmd(int cmd, bool absolute){
    vector<int32_t> cmdList;
    for(int i = 0; i < this->cableNumber; i++){ cmdList.push_back(cmd); }
    return this->MoveAllMotorCmd(cmdList, absolute);
}

void CableController::CalibrationMotor(int index, int32_t currentCmdPos){
    if(!this->useCable) return;
    assert(index >= 0 && index <= cableNumber);
        motorNode.SetCurrentPosn(index, currentCmdPos);
}

double CableController::GetMotorPosMeasured(int index){
    if(!this->useCable) return 0;
        assert(index >= 0 && index <= cableNumber);
        return (double) motorNode.GetCurrentPosn(index);
}

double CableController::GetMotorTorqueMeasured(int index){
    if(!this->useCable) return 0;
    assert(index >= 0 && index <= cableNumber);
        return (double) motorNode.GetTrqMeasured(index);
}
