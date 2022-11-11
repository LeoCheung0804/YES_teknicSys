#include <Windows.h>
#include <string>
#include <iostream>

using namespace std;
class ArduinoBLENode{
private:
    char* ComPortName;
    unsigned char Ard_char[8] = {'(','o',',',' ',' ',' ',' ',')'};
    HANDLE hComm;
    
    // set communication port
    bool SetSerialParams(HANDLE hComm){
        // Set parameters for serial port
        DCB dcbSerialParams = { 0 }; // Initializing DCB structure
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        bool Status = GetCommState(hComm, &dcbSerialParams); // retreives  the current settings
        if (Status == false){ cout << "Error in GetCommState()\n"; return false; }

        dcbSerialParams.BaudRate = CBR_57600;// Setting BaudRate
        dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
        dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
        dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None

        SetCommState(hComm, &dcbSerialParams);
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

        if (SetCommTimeouts(hComm, &timeouts) == FALSE){ cout << "Error! in Setting Time Outs\n"; return false; }
        // Set recieve mask                
        if (!(bool)SetCommMask(hComm, EV_RXCHAR)){ cout << "Error! in Setting CommMask\n"; }
        
        return true;
    }
public:
    ArduinoBLENode(char* portName){
        this->ComPortName = portName;
    }

    // Connect Node
    bool Connect(){
        hComm = CreateFile(ComPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hComm == INVALID_HANDLE_VALUE){ cout << "Error: " << ComPortName << " cannot be opened.\n"; }
        else { cout << ComPortName << " opened.\n"; }
        if (!SetSerialParams(hComm)) { return false; }
        return true;
    }
};