#include <iostream>
#include <string>
#include "..\Dependencies\sFoundation20\inc\pubSysCls.h"

using namespace std;
using namespace sFnd;

class TeknicNode{
private:
    int motorNumber;
public:
    TeknicNode(int motorNumber){this->motorNumber = motorNumber;}
    bool Connect(){
        SysManager* myMgr = SysManager::Instance();
        // Start the programme, scan motors in network
        try{
            if (CheckMotorNetwork() < 0){
                cout << "Motor network not available. Exit programme." << endl;
                return -1;
            }
        }
        catch(mnErr& theErr) {    //This catch statement will intercept any error from the Class library
            printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");  
            printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return -1;
        }
    }
}