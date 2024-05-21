// Copyright 2020 Feiler
#pragma once
#include <ros/ros.h>
#include <arpa/inet.h>
#include <thread>
#include "udp_sender.h"

#define MAXLINE 100*1024

namespace tod_network {

class UdpReceiver {
public:
    explicit UdpReceiver(const int destPort);
    ~UdpReceiver() { close(sockfd); }

    template <class Msg>
    int receive_ros_msg(Msg& msg) {
        char buffer[MAXLINE];
        int recvBytes = recv(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL);
        try {
            // Reconstruct ros message object
            ros::serialization::IStream stream((uint8_t*) buffer, recvBytes);
            ros::serialization::Serializer<Msg>::read(stream, msg);
        } catch (const ros::serialization::StreamOverrunException& e ) {
            ROS_DEBUG("Error during message reconstruction");
        }
        return recvBytes;
    }

    int receive_data(char *msg) { return recv(sockfd, msg, MAXLINE, MSG_WAITALL); }
    int wait_for_udp_receiver_to_close();

private:
    struct sockaddr_in servaddr;
    int sockfd;
    std::thread anti_block_thread;
    UdpSender anti_block_sender;

    void send_data_for_anti_block();
};
}; //namespace tod_network
