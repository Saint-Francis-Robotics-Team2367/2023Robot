#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <thread>
#include <string>
#include <iostream>

class PiModule {
    public:
    std::string ID;
    //std::thread pi_thread;
    bool stop_thread;
    std::thread pi_thread;
    
   PiModule(int inputID) {
        ID = std::to_string(inputID);
        stop_thread = false;
        //std::cout << "check 0";
        pi_thread = std::thread(&PiModule::start_server, this);
    } //EVERYTHING NEEDS TO BE NONSTATIC

    void start_server(); // Run this function once and ur good to go (its threaded)
    double current_val;
    double get_distance();
    void run_update();
    bool stop_server();

    int sockfd_g;
    char* buffer_g;
    struct sockaddr_in servaddr, cliaddr_g;



};