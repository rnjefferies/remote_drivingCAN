// Copyright 2020 Feiler
#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_network/udp_receiver.h>
#include <tod_msgs/PaketInfo.h>
#include <memory>

namespace tod_network {

template <typename T>
class Receiver {
private:
    struct RosMsgReceiver{
        ros::Publisher recvMsgPubs, recvMsgPaketInfoPubs;
        std::unique_ptr<tod_network::UdpReceiver> udpReceiver{nullptr};
        bool printedInfo{false};
        bool restamp{false};
        std::unique_ptr<std::thread> thread{nullptr};
        T msg;
        tod_msgs::PaketInfo paketInfoMsg;
    };

public:
    Receiver(ros::NodeHandle &n, int port, bool restamp = false) : _n(n) {
        add_processer("/received_topic", port, restamp);
    }
    explicit Receiver(ros::NodeHandle &n) : _n(n) {}

    void add_processer(const std::string &topic, const int port, const bool restamp = false) {
        auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
        receiver->udpReceiver = std::make_unique<tod_network::UdpReceiver>(port);
        receiver->recvMsgPubs = _n.advertise<T>(topic, 1);
        receiver->recvMsgPaketInfoPubs = _n.advertise<tod_msgs::PaketInfo>(topic + "_paket_info", 1);
        receiver->restamp = restamp;
    }

    void receive() {
        for (auto receiver : _rosMsgReceivers)
            receiver->thread = std::make_unique<std::thread>(&Receiver::receive_msgs, this, receiver);
    }

    void run() {
        receive();
        ros::spin();
    }


private:
    ros::NodeHandle &_n;
    std::vector<std::shared_ptr<RosMsgReceiver>> _rosMsgReceivers;

    void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver) {
        while (ros::ok()) {
            int sizeByte = receiver->udpReceiver->receive_ros_msg(receiver->msg);

            // publish paket info msg
            receiver->paketInfoMsg.header.stamp = ros::Time::now();
            receiver->paketInfoMsg.seqNum = receiver->msg.header.seq;
            receiver->paketInfoMsg.sizeBit = sizeByte * 8;
            receiver->paketInfoMsg.latencyUsec = int64_t(ros::Time::now().toNSec()/1000
                                                         - receiver->msg.header.stamp.toNSec()/1000);
            receiver->recvMsgPaketInfoPubs.publish(receiver->paketInfoMsg);

            // publish restamped data msg
            if (receiver->restamp)
                receiver->msg.header.stamp = ros::Time::now();
            receiver->recvMsgPubs.publish(receiver->msg);
            if (!receiver->printedInfo) {
                receiver->printedInfo = true;
                ROS_INFO("%s: receiving topic %s", ros::this_node::getName().c_str(),
                         receiver->recvMsgPubs.getTopic().c_str());
            }
        }
    }
};

}; // namespace tod_network
