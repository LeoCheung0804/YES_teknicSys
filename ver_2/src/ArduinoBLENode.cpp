#include "..\include\ArduinoBLENode.h"
#include <iostream>
#include <thread>

ArduinoBLENode::ArduinoBLENode(){}

bool ArduinoBLENode::SetSerialParams(){
    // Set parameters for serial port
    DCB dcbSerialParams = { 0 }; // Initializing DCB structure
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    bool Status = GetCommState(this->hComm, &dcbSerialParams); // retreives  the current settings
    if (Status == false){ cout << "Error in GetCommState()\n"; return false; }

    dcbSerialParams.BaudRate = CBR_57600;// Setting BaudRate
    dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
    dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
    dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None

    SetCommState(this->hComm, &dcbSerialParams);
    if (Status == false){ cout << "Error! in Setting DCB Structure\n"; return false; }
    else{
        printf("   Setting DCB Structure Successful\n");
        printf("       Baudrate = %d\n", dcbSerialParams.BaudRate);
        printf("       ByteSize = %d\n", dcbSerialParams.ByteSize);
        printf("       StopBits = %d\n", dcbSerialParams.StopBits);
        printf("       Parity   = %d\n", dcbSerialParams.Parity);
        cout << endl;
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(this->hComm, &timeouts) == FALSE){ cout << "Error! in Setting Time Outs\n"; return false; }
    // Set recieve mask                
    if (!(bool)SetCommMask(this->hComm, EV_RXCHAR)){ cout << "Error! in Setting CommMask\n"; }
    
    return true;
}

bool ArduinoBLENode::Connect(string portName){
    cout << "Connecting to BLE Device on port: " << portName << endl;;
    const char* cArray = portName.c_str();
    this->hComm = CreateFile(cArray, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (this->hComm == INVALID_HANDLE_VALUE){ 
        cout << "Error: " << portName << " cannot be opened.\n"; 
    } else { 
        cout << portName << " opened.\n"; 
    }
    if (!SetSerialParams()) { 
        return false; 
    }
    cout << "Connected to BLE Device on port: " << portName << endl;
    return true;
}

void ArduinoBLENode::Disconnect(){
    this->Send("(z,    )");
}

string ArduinoBLENode::Send(string Ard_char){
    char buffer[8] = {};
    for(int i = 0; i < 8; i++){
        buffer[i] = Ard_char[i];
    }
    DWORD dNoOfBytesWritten = 0;
    if (!(bool)WriteFile(this->hComm, buffer, 8, &dNoOfBytesWritten, NULL)){ 
        cout << "Arduino writing error: " << GetLastError() << endl; 
    }    
    return Read();
    // Read(); // need this?
}

string ArduinoBLENode::Read(){
    DWORD dwEventMask, BytesRead;
    int i{0};
    char tmp;
    string msg = "";

    bool Status = WaitCommEvent(this->hComm, &dwEventMask, NULL); // wait till brick is ready from ABB
    if (Status == false){ cout << "Error in setting WaitCommEvent()\n";} //quitType = 'q'; return; }
    else {
        auto start = chrono::steady_clock::now();
        auto end = chrono::steady_clock::now();
        while(chrono::duration_cast<chrono::milliseconds>(end-start).count()  < 1000){  
            ReadFile(this->hComm, &tmp, sizeof(tmp), &BytesRead, NULL);
            if(BytesRead > 0){
                msg.push_back(tmp);
                break;
            }
            end = chrono::steady_clock::now();
        }
        while(BytesRead > 0){
            ReadFile(this->hComm, &tmp, sizeof(tmp), &BytesRead, NULL);
            msg.push_back(tmp);
            i++;
        }
    }
    return msg;

}