// Copyright 2020 Feiler
#pragma once
#include <ros/ros.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <arpa/inet.h>
#include "connection_configs.h"

namespace tod_network {

class UdpSender {
public:
    UdpSender(const std::string& destIPAddress, const int& destPort);
    ~UdpSender() { close(sockfd); }

    template <class Msg>
    int send_ros_msg(const Msg& msg) {
        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(msg);
        return sendto(sockfd, serMsg.message_start, serMsg.num_bytes, 0,
                      (struct sockaddr *)&servaddr, sizeof(servaddr));
    }

    template <class Cfg>
    int send_ros_config(const Cfg& cfg) {
        dynamic_reconfigure::Config msg;
        cfg.__toMessage__(msg);
        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(msg);
        return sendto(sockfd, serMsg.message_start, serMsg.num_bytes, 0,
                      (struct sockaddr *)&servaddr, sizeof(servaddr));
    }

    int send(const char *msg, size_t size) {
        return sendto(sockfd, msg, size, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
    }

    void change_destination(const std::string& destIPAddress, const int& destPort = -1);
    void print_connection_specs();

private:
    struct sockaddr_in servaddr;
    int sockfd;
};
}; // namespace tod_network
