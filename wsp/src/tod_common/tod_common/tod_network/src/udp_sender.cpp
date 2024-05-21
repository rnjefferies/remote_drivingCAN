// Copyright 2020 Feiler
#include "tod_network/udp_sender.h"

namespace tod_network {

UdpSender::UdpSender(const std::string& destIPAddress, const int& destPort) {
    change_destination(destIPAddress, destPort);
    // create sender socket file descriptor
    if ( (sockfd = socket(servaddr.sin_family, SOCK_DGRAM, 0)) < 0 ) {
        perror("cannot create socket");
        printf("%s:%i", destIPAddress.c_str(), destPort);
        exit(EXIT_FAILURE);
    }
}

void UdpSender::change_destination(const std::string &destIPAddress, const int &destPort) {
    // fill servaddr with values
    inet_aton(destIPAddress.c_str(), &(servaddr.sin_addr)); // ip
    if (destPort >= 0) servaddr.sin_port = htons(destPort); // port
    servaddr.sin_family = AF_INET;
}

void UdpSender::print_connection_specs() {
    printf("Destination IP Address %s\n", inet_ntoa(servaddr.sin_addr));
    auto port = ntohs(servaddr.sin_port);
    printf("Destination Port %i\n", port);
}
}; // namespace tod_network
