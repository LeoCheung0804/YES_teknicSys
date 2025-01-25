
#include "..\include\DeltaClient.h"

DeltaClient::DeltaClient(bool isOnline, bool useRail){ 
    // init Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cout << "Failed to initialize Winsock" << endl;
    }

    // Create UDP socket
    SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udpSocket == INVALID_SOCKET) {
        cout << "Failed to create socket" << endl;
        system("pause");
    }

    // Set up server address
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12345); // Use appropriate port number
    serverAddr.sin_addr.s_addr = inet_addr("192.168.0.1"); // Use RPi's IP address

    this->serverAddr = serverAddr;
    this->udpSocket = udpSocket;
    this->useRail = useRail; 
}

void DeltaClient::Connect(int motorPortNumber, int railNumber){
    if(!this->useRail) return;
    this->railNumber = railNumber;
    this->bArry = new bool(railNumber);
    
    const char* request = "connect";
    
    if (sendto(this->udpSocket, request, strlen(request), 0, (sockaddr*)&this->serverAddr, sizeof(this->serverAddr)) == SOCKET_ERROR) {
        cout << "Failed to send request to Delta Server" << endl;
        closesocket(this->udpSocket);
        system("pause");
    }

    // Receive response
    char buffer[1024];
    int serverAddrLen = sizeof(this->serverAddr);
    int bytesReceived = recvfrom(this->udpSocket, buffer, sizeof(buffer), 0, (sockaddr*)&this->serverAddr, &serverAddrLen);
    
    if (bytesReceived == SOCKET_ERROR) {
        cout << "Failed to receive response from Delta Server" << endl;
        closesocket(this->udpSocket);
        system("pause");
        this->isConnected = false;
        return;
    }

    cout << "Rail Controller Online." << endl;
    this->isConnected = true;
}

void DeltaClient::Disconnect(){
    if(!this->useRail) return;
    WSACleanup();
    this->isConnected = false;
}

bool DeltaClient::IsConnected(){ return this->isConnected; }

void DeltaClient::SelectWorkingMotor(int index){
    if(!this->useRail) return;
    this->bArry[workingMotor] = false;
    this->bArry[index] = true;
    workingMotor = index;
}

void DeltaClient::MoveSelectedMotorCmd(double cmd, bool absulote){
    if(!this->useRail) return;
    if(!absulote) return;
    const char* request = "move";
    if (sendto(this->udpSocket, request, strlen(request), 0, (sockaddr*)&this->serverAddr, sizeof(this->serverAddr)) == SOCKET_ERROR) {
        cout << "Failed to send request to Delta Server" << endl;
        closesocket(this->udpSocket);
        system("pause");
    }

    // // Receive response
    // char buffer[1024];
    // int serverAddrLen = sizeof(this->serverAddr);
    // int bytesReceived = recvfrom(this->udpSocket, buffer, sizeof(buffer), 0, (sockaddr*)&this->serverAddr, &serverAddrLen);
        
}

void DeltaClient::CalibrationMotor(int index, double currentCmdPos){
    cout << "Caution !!!!!! CalibrationMotor not implemented" << endl;
    return;
    // if(!this->useRail) return;
    // bool *busyFlag = new bool(this->railNumber);
    // bool *homeFlag = new bool(this->railNumber);
    
    // for(int i = 0; i < this->railNumber; i++){
    //     busyFlag[i] = false;
    //     homeFlag[i] = false;
    // }
    // this->motorNode.WriteReq("MAIN.Axis_GoalPos", currentCmdPos); // write "MAIN.Axis1_GoalPos"
    // homeFlag[index] = true; // signal targeted rail motor for homing
    // this->motorNode.WriteReq("MAIN.bHomeSwitch", homeFlag); // write "MAIN.bHomeSwitch"
    // homeFlag[index] = false; // return to false
    // busyFlag[index] = false;

    // while(!busyFlag[index]){ // wait for motor busy flag on, ie. update current pos started
    //     cout << "Waiting for motor " << index << " to home" << endl;
    //     this->motorNode.ReadReq("MAIN.Axis_Home.Busy", busyFlag);
    // }
    // while(busyFlag[index]){ // wait for motor busy flag off, ie. completed updated position
    //     cout << "Waiting for motor " << index << " to home" << endl;
    //     this->motorNode.ReadReq("MAIN.Axis_Home.Busy", busyFlag);
    // }
}

vector<int> DeltaClient::GetMotorPosMeasured(){
    cout << "Caution !!!!!! GetMotorPosMeasured not implemented" << endl;
    return vector<int>();
    // long nErr;
    // bool *actPos = new bool(this->railNumber);
    // this->motorNode.ReadReq("MAIN.actPos", actPos);
    // vector<int> result(actPos, actPos + sizeof(actPos) / sizeof(actPos[0]));
    // return result;
}