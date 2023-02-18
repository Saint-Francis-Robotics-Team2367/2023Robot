#include <stdio.h>
#include <iostream>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <cassert>
#include <DriveBaseModule.h>

#define MAXBUFSIZE 1024
#define BCAST_PORT 5800
#define MAXLINE 1024

class Vision {
    // public constants and variables
    public: 
    int socketID = 0;
    char receiveData[MAXBUFSIZE] = {0};

    // constructor
    Vision(int port);

    // public methods
    public: 
    int ReceivePacket();
    int ReceiveInt();
    int ReceiveCoords(int &x, int &y);
    int SwitchCameraStream();
    void SetupClient(int port);
    void SetupServer();
    void broadcast(int socket, const char *mess);


};