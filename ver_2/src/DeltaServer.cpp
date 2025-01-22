#define WIN32_LEAN_AND_MEAN  // Add this to reduce Windows header conflicts
#include <windows.h>         // Add this first
#include <WinSock2.h>
#include <WS2tcpip.h>
#include "..\include\RailController.h"
#include "..\tools\json.hpp"
#include <string>
#include <iostream>  // Add this for std::cout, std::cerr, std::endl

#pragma comment(lib, "Ws2_32.lib")

using json = nlohmann::json;

class DeltaServer {
private:
    SOCKET socket_;
    enum { max_length = 65507 }; // Maximum UDP packet size
    char data_[max_length];
    RailController rail;

public:
    DeltaServer(unsigned short port) {
        // Initialize Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }

        // Create socket
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == INVALID_SOCKET) {
            WSACleanup();
            throw std::runtime_error("Socket creation failed");
        }

        // Bind socket
        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(port);
        serverAddr.sin_addr.s_addr = INADDR_ANY;

        if (::bind(socket_, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
            closesocket(socket_);
            WSACleanup();
            throw std::runtime_error("Bind failed");
        }
    }

    ~DeltaServer() {
        closesocket(socket_);
        WSACleanup();
    }

    void start() {
        std::cout << "Connecting to Rail Controller..." << std::endl;
        
        // Init Rail Motor Nodes
        this->rail = RailController(false, true);
        this->rail.Connect(851, 1);
        if(!this->rail.IsConnected()){
            std::cout << "Error: Rail motors not connected." << std::endl;
            std::cout << "Exiting ..." << std::endl;
            exit(-1);
        }

        std::cout << "Server is listening..." << std::endl;
        
        while (true) {
            sockaddr_in clientAddr;
            int clientAddrLen = sizeof(clientAddr);

            int bytesReceived = recvfrom(socket_, 
                                       data_, 
                                       max_length, 
                                       0, 
                                       (SOCKADDR*)&clientAddr, 
                                       &clientAddrLen);

            if (bytesReceived == SOCKET_ERROR) {
                std::cerr << "recvfrom failed: " << WSAGetLastError() << std::endl;
                continue;
            }

            try {
                std::string json_str(data_, bytesReceived);
                json j = json::parse(json_str);
                handle_json_message(j);
            }
            catch (const json::parse_error& e) {
                std::cerr << "JSON parsing error: " << e.what() << std::endl;
            }
        }
    }

private:
    void handle_json_message(const json& j) {
        // Example of processing the JSON message
        std::cout << "Received JSON message: " << j.dump(2) << std::endl;
        try{
            int index = j["index"];
            int command = j["command"];
            switch(command) {
                case 0: {  // Add braces to create a new scope
                    double position = j["position"];
                    this->rail.SelectWorkingMotor(index);
                    this->rail.MoveSelectedMotorCmd(position);
                    break;
                }
                default:
                    std::cout << "Invalid command" << std::endl;
                    break;
            }
        }catch(const std::exception& e){
            std::cout << "Error: " << e.what() << std::endl;
        }
    }
};

int main() {
    try {
        DeltaServer server(12345); // Listen on port 12345
        server.start();
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
