#include <iostream>
#include <string>
#include <vector>
#include "..\Dependencies\sFoundation20\inc\pubSysCls.h"

using namespace std;
using namespace sFnd;

class TeknicNode{
private:
    int motorNumber;
    unsigned int portCount;
    vector<string> comHubPorts;
    vector<INode*> nodeList; // create a list for each node
    bool CheckMotorNetwork() {
        SysManager* myMgr = SysManager::Instance();

        sFnd::SysManager::FindComHubPorts(comHubPorts);

        cout << "Found " <<comHubPorts.size() << " SC Hubs\n";
        for (portCount = 0; portCount < comHubPorts.size(); portCount++) {
            myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());
        }
        if (portCount < 0) {
            cout << "Unable to locate SC hub port\n";
            return false;
        }
        if(portCount==0) { return false; } // do we need this?
        
        myMgr->PortsOpen(portCount);
        int connectedMotor = 0;
        for (int i = 0; i < portCount; i++) { // check no. of nodes in each ports
            IPort &myPort = myMgr->Ports(i);
            connectedMotor += myPort.NodeCount();
        }
        printf("Total %d/%d motor connected.\n", connectedMotor, motorNumber);
        if(motorNumber != connectedMotor){
            cout << "Error: Connected motor number wrong!" << endl;
            return false;
        }
        for (int i = 0; i < portCount; i++) { // check no. of nodes in each ports
            IPort &myPort = myMgr->Ports(i);
            // myPort.BrakeControl.BrakeSetting(0, BRAKE_AUTOCONTROL); // do we need this?
            // myPort.BrakeControl.BrakeSetting(1, BRAKE_AUTOCONTROL);
            printf(" Port[%d]: state=%d, nodes=%d\n", myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
        
            for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {
                INode &theNode = myPort.Nodes(iNode);
                theNode.EnableReq(false); //Ensure Node is disabled before loading config file
                myMgr->Delay(200);

                printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
                printf("            userID: %s\n", theNode.Info.UserID.Value());
                printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
                printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
                printf("             Model: %s\n", theNode.Info.Model.Value());

                theNode.Status.AlertsClear();               //Clear Alerts on node 
                theNode.Motion.NodeStopClear();             //Clear Nodestops on Node               
                theNode.EnableReq(true);                    //Enable node 
                theNode.Motion.PosnMeasured.AutoRefresh(true);
                theNode.Motion.TrqMeasured.AutoRefresh(true);
                printf("Node %d enabled. ", iNode);

                theNode.AccUnit(INode::RPM_PER_SEC);        //Set the units for Acceleration to RPM/SEC
                theNode.VelUnit(INode::RPM);                //Set the units for Velocity to RPM
                theNode.Motion.AccLimit = 40000;           //100000 Set Acceleration Limit (RPM/Sec)
                theNode.Motion.NodeStopDecelLim = 5000;
                theNode.Motion.VelLimit = 3000;             //700 Set Velocity Limit (RPM)
                theNode.Info.Ex.Parameter(98, 1);           //enable interrupting move
                theNode.Motion.Adv.TriggerGroup(1);         //Set all nodes trigger group num. as 1
                cout << "AccLimit and VelLimit set." << endl;

                nodeList.push_back(&theNode);               // add node to list

                double timeout = myMgr->TimeStampMsec() + 2000; //TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
                //This will loop checking on the Real time values of the node's Ready status
                while (!theNode.Motion.IsReady()) {
                    if (myMgr->TimeStampMsec() > timeout) {
                        printf("Error: Timed out waiting for Node %d to enable\n", iNode);
                        return false;
                    }
                }
            }
        }
        return true;
    }
public:
    TeknicNode(int motorNumber){this->motorNumber = motorNumber;}
    bool Connect(){
        SysManager* myMgr = SysManager::Instance();
        // Start the programme, scan motors in network
        try{
            if (!CheckMotorNetwork()){
                cout << "Motor network not available. Exit programme." << endl;
                return false;
            }
            else{
                return true;
            }
        }
        catch(mnErr& theErr) {    //This catch statement will intercept any error from the Class library
            printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");  
            printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return false;
        }
    }
};