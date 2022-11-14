#ifndef TeknicNode_H
#define TeknicNode_H
#include "..\Dependencies\sFoundation20\inc\pubSysCls.h"
#include <vector>
#include <string>

using namespace std;
using namespace sFnd;
class TeknicNode{
private:
    int nodeNumber;
    unsigned int portCount;
    vector<string> comHubPorts;
    vector<INode*> nodeList; // create a list for each node
    SysManager* myMgr;
    /// @brief Scan the network, find all connected teknic motors.
    /// @return bool. true if there are exact number of teknic motors connected.
    bool CheckMotorNetwork();
public:
    TeknicNode();
    /// @brief Init Teknic Motor Communication Node By Motor number
    /// @param nodeNumber Total Teknic motor node needed.
    TeknicNode(int nodeNumber);
    /// @brief Set total node number
    /// @param nodeNumber 
    /// @return Total Teknic motor node needed.
    int SetNodeNumber(int nodeNumber);
    /// @brief Scan the network, find all connected sc hub and all connected teknic motors.
    /// @return bool. Whether the connection has been established.
    bool Connect(); 
    INode* GetNode(int nodeIndex);
    vector<INode*> GetNodeList();
    bool Disconnect();

};

#endif
