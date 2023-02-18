#include "Vision.h"

using namespace std;

Vision::Vision(int port = BCAST_PORT){
    int srcaddrSize;
    struct sockaddr_in localUdp;
    const int bCastPort = port;
    struct sockaddr_in serv_addr;

    //setup udp socket
    if ((socketID = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        perror("UDP socket failed");
    }

    if (setsockopt(socketID, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &srcaddrSize, sizeof(srcaddrSize)))
    {
        perror("setsockopt");
    }

    memset(&localUdp, 0, sizeof(serv_addr));
    localUdp.sin_family = AF_INET;
    localUdp.sin_addr.s_addr = INADDR_ANY;
    localUdp.sin_port = htons( bCastPort );

    if (bind(socketID, (struct sockaddr *)&localUdp,sizeof(localUdp))) {
        perror("UDP bind to port failed");
    }

};

int Vision::ReceivePacket(){
    struct sockaddr_in bCastRecv;
	socklen_t addrlen = sizeof(struct sockaddr_in);
	ssize_t packetSize;

	std::cout << "Waiting for broadcast..." <<std::endl;
	memset(&bCastRecv, 0, sizeof(bCastRecv));
	memset(receiveData, 0, MAXBUFSIZE);
	packetSize = recvfrom(socketID, receiveData, MAXBUFSIZE, 0, (struct sockaddr *) &bCastRecv, &addrlen);

	cout<<"Packet Size: "<< packetSize <<endl;
	cout<<"Packet: "<< receiveData <<endl; 
    
	return packetSize;

/*
    do{
        std::cout << "Waiting for broadcast..." <<std::endl;
        memset(&bCastRecv, 0, sizeof(bCastRecv));
        memset(buffer, 0, MAX_BUFF);
        currPacket = recvfrom(socketID, buffer, MAX_BUFF, 0, (struct sockaddr *) &bCastRecv, &addrlen);
        cout<<"Packet Size: "<< currPacket <<endl;
        cout<<"Packet: "<< buffer <<endl;
    } while(currPacket != -1);
*/
}

int Vision::ReceiveInt(){
	int dataLen = ReceivePacket();
    
	if (dataLen != -1) {
		return atoi(receiveData);
	} else {
		return -1;
	}
}

int Vision::ReceiveCoords(int &x, int &y) {
	int dataLen = ReceivePacket();

	if (dataLen != -1) {
		std::string xtoken, ytoken;
		std::string delimiter = ":";
		std::string msg = string(receiveData); 
		xtoken = msg.substr(0, msg.find(delimiter)); 
		ytoken = msg.erase(0, msg.find(delimiter) + delimiter.length()); 
		x = atoi(xtoken.c_str()); 
		y = atoi(ytoken.c_str()); 
		return dataLen;
	} else {
		return -1;
	}

}