#include "..\include\CableController.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <assert.h>
#include <Windows.h>
#include <conio.h>
#include <queue>

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
    this->logger.OpenFile("log\\trq.log");
    for(int i = 0; i < brakeNumber; i++) {
        this->brakeOnFlags[i] = false;
        cout << this->brakeOnFlags[i] << endl;
    }
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
    this->logger.CloseFile();
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

void CableController::SetCableTrqByIndex(int index, float targetTrq, float tolerance){
    cout << "Setting all cable torque to: " << targetTrq << endl;
    // this->OpenBrake(int(index/2));
    if(isOnline && this->useMotor){
        cout << "current torques: " << endl;
        nodeList[index]->Motion.AccLimit = 200;
        // get current trq
        float currentTrq = nodeList[index]->Motion.TrqMeasured.Value();
        cout << "motor: " << index << ": " << currentTrq << "       " << endl;
        
        if(currentTrq > targetTrq){ 
            nodeList[index]->Motion.MoveVelStart(-10); 
            currentTrq = nodeList[index]->Motion.TrqMeasured.Value();
            while(currentTrq > targetTrq - tolerance){
                cout << "current torques: " << endl;
                // get current trq
                currentTrq = nodeList[index]->Motion.TrqMeasured.Value();
                cout << "motor: " << index << ": " << currentTrq << "       " << endl;
                if(kbhit()){
                    nodeList[index]->Motion.NodeStop(STOP_TYPE_ABRUPT);
                    break;
                }
            }
        }
        nodeList[index]->Motion.NodeStop(STOP_TYPE_ABRUPT);
        nodeList[index]->Motion.AccLimit = 40000;
        
    }
    // this->CloseBrake(int(index/2));
    cout << "Set cable " << index << " torque to: " << targetTrq << " finished." << endl;
    
}

/*
void CableController::SetCableTrq(float targetTrq, float tolerance){
    cout << "Setting all cable torque to: " << targetTrq << endl;
    this->OpenAllBrake();
    if(isOnline && this->useMotor){
        bool moving = true;
        int index = 0;
        cout << "current torques: " << endl;
        for(INode* node : nodeList){
            node->Motion.AccLimit = 200;
            // get current trq
            float currentTrq = node->Motion.TrqMeasured.Value();
            cout << "motor: " << index << ": " << currentTrq << "       " << endl;
            // check current trq and set move direction
            index++;
        }
        int windowSize = 5;
        vector<queue<float>> bucketList;
        for(int i = 0; i < cableNumber; i++){
            queue<float> bucket;
            bucketList.push_back(bucket);
        }
        float result = 0;
        // -5 < trq < -3.5
        while(moving){
            printf("\x1b[9A");
            moving = false;
            cout << "current torques: " << endl;
            int index = 0;
            string log = "";
            for(INode* node : nodeList){
                // get current trq
                float currentTrq = node->Motion.TrqCommanded.Value();
                bucketList[index].push(currentTrq);
                if(bucketList[index].size() > windowSize){
                    bucketList[index].pop();
                }
                result = 0;
                for(int i = 0; i < bucketList[index].size(); i++){
                    result += bucketList[index].front();
                    bucketList[index].push(bucketList[index].front());
                    bucketList[index].pop();
                }
                result = result / bucketList[index].size();
                if(result > targetTrq){ 
                    node->Motion.MoveVelStart(-10); 
                    moving = true;
                }
                else if (result < targetTrq - tolerance){ 
                    node->Motion.MoveVelStart(10);
                    moving = true;
                }
                cout << "motor: " << index << ": " << result << "       " << endl;
                // check current trq
                if(result > targetTrq || result < targetTrq - tolerance)
                    moving = true; // continue loop 
                    index ++;
                log +=  to_string(result) + ",";
            }
            log += "\n";
            logger.LogInfo(log);
            if(kbhit()){
                break;
            }
        }

        for(INode* node : nodeList){
            node->Motion.NodeStop(STOP_TYPE_ABRUPT);
            node->Motion.AccLimit = 40000;
        }
        
    }
    this->CloseAllBrake();
    cout << "Set all cable torque to: " << targetTrq << " finished." << endl;
}
*/


void CableController::SetCableTrq(float targetTrq, float tolerance){
    cout << "Setting all cable torque to: " << targetTrq << endl;
    this->OpenAllBrake();
    if(isOnline && this->useMotor){
        bool* stoppFlag = new bool(this->cableNumber);
        bool moving = true;
        int index = 0;
        cout << "current torques: " << endl;
        float currentTrq = 0;
        for(INode* node : nodeList){
            node->Motion.AccLimit = 200;
            // get current trq
            currentTrq = node->Motion.TrqMeasured.Value();
            cout << "motor: " << index << ": " << currentTrq << "       " << endl;
            // check current trq and set move direction
            if(currentTrq > targetTrq - tolerance){ 
                node->Motion.MoveVelStart(-10); 
                stoppFlag[index] = false;
            }
            index++;
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
                index = 0;
                log = "";
                for(INode* node : nodeList){
                    // get current trq
                    currentTrq = node->Motion.TrqMeasured.Value();
                    if (currentTrq < targetTrq - tolerance || stoppFlag[index]){ 
                        node->Motion.NodeStop(STOP_TYPE_ABRUPT);
                        stoppFlag[index] = true;
                    }else{
                        moving = true;
                    }
                    cout << "motor: " << index << ": " << currentTrq << "       " << endl;
                    index ++;
                    log +=  to_string(currentTrq) + ",";
                }
                logger.LogInfo(log);
            }
            if(kbhit()){
                for(INode* node : nodeList){
                    node->Motion.NodeStop(STOP_TYPE_ABRUPT);
                    node->Motion.AccLimit = 40000;
                }
                moving = false;
                break;
            }
        }

        for(INode* node : nodeList){
            node->Motion.NodeStop(STOP_TYPE_ABRUPT);
            node->Motion.AccLimit = 40000;
        }
        
    }
    this->CloseAllBrake();
    cout << "Set all cable torque to: " << targetTrq << " finished." << endl;
}

void CableController::SetTrqLmt(float lmt){
    this->absTrqLmt = lmt;
}

void CableController::ClearAlert(){
    for (INode* node : nodeList) {      
        node->EnableReq(true);                    //Enable node 
        node->Status.AlertsClear();               //Clear Alerts on node 
        node->Motion.NodeStopClear();             //Clear Nodestops on Node     
    }
}

void CableController::StopAllMotor(){
    for(INode* node : nodeList){
        node->Motion.NodeStop(STOP_TYPE_ABRUPT);
    } 
}

bool CableController::MoveSingleMotorCmd(int index, int32_t cmd, bool absolute){
    assert(index >= 0 && index <= cableNumber);
    // assert(!this->brakeOnFlags[index/2]);
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
            this->eStop = true;
            return false;
        }
    }else{
        Sleep(10);
    }
    return true;
}

bool CableController::MoveAllMotorCmd(vector<int32_t> cmdList, bool absolute){
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
    assert(index >= 0 && index <= cableNumber);
    if(this->isOnline && this->useMotor){
        nodeList[index]->Motion.PosnMeasured.Refresh();
        nodeList[index]->Motion.AddToPosition(-nodeList[index]->Motion.PosnMeasured.Value() + currentCmdPos);
    }else{
        cout << "Offline mode, skip calibration" << endl;
    }
}

double CableController::GetMotorPosMeasured(int index){
    if(this->isOnline && this->useMotor){
        assert(index >= 0 && index <= cableNumber);
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
