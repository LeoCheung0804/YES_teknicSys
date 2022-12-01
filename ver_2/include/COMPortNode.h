#ifndef COMPortNode_H
#define COMPortNode_H

#include <Windows.h>
#include <string>

using namespace std;
class COMPortNode{
private:
    bool isOnline;
    HANDLE hComm;
    int readTimeout=1000;
    
    // set communication port
    bool SetSerialParams();
public:
    COMPortNode(bool isOnline=false);

    bool Connect(string portName);  
    void Disconnect();

    string Read();
    string Send(string Ard_char);
};
#endif