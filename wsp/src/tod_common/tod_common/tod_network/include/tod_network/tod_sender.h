// Copyright 2020 Feiler
#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_network/udp_sender.h>
#include <memory>
#include <algorithm>

namespace tod_network {

template <typename T>
class Sender {
private:
    struct RosMsgSender {
        ros::Subscriber sendMsgSubs;
        std::unique_ptr<tod_network::UdpSender> udpSender{nullptr};
        bool printedInfo{false};
    };

public:
    Sender(ros::NodeHandle &n, int port, bool senderInVehicle)
        : _n(n), _senderInVehicle(senderInVehicle) {
        std::string statusTopic =
            (_senderInVehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
        _statusSubs = _n.subscribe(statusTopic, 1, &Sender::statusMessageReceived, this);
        add_processer("/topic_to_send", port);
    }
    Sender(ros::NodeHandle &n, bool senderInVehicle)
        : _n(n), _senderInVehicle(senderInVehicle) {
        std::string statusTopic =
            (_senderInVehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
        _statusSubs = _n.subscribe(statusTopic, 1, &Sender::statusMessageReceived, this);
    }

    void run() const { ros::spin(); }

    void add_processer(const std::string &topic, const int port) {
        auto sender = _rosMsgSenders.emplace_back(std::make_shared<RosMsgSender>());
        sender->udpSender = std::make_unique<tod_network::UdpSender>(_receiverIp, port);
        sender->sendMsgSubs = _n.subscribe<T>(
            topic, 10, boost::bind(&Sender::sendMessageReceived, this, _1, sender));
    }

    void send_in_control_mode(const uint8_t mode) {
        _sendingControlModes.push_back(mode);
    }

private:
    std::string _receiverIp{"127.0.0.1"};
    std::vector<std::shared_ptr<RosMsgSender>> _rosMsgSenders;
    uint8_t _connectionStat{tod_msgs::Status::TOD_STATUS_IDLE};
    uint8_t _controlMode;
    ros::Subscriber _statusSubs;
    ros::NodeHandle &_n;
    bool _senderInVehicle;
    std::vector<uint8_t> _sendingControlModes;

    bool connected() {
        return _connectionStat != tod_msgs::Status::TOD_STATUS_IDLE;
    }

    void statusMessageReceived(const tod_msgs::Status &msg) {
        _connectionStat = msg.tod_status;
        _controlMode = _senderInVehicle ? msg.vehicle_control_mode : msg.operator_control_mode;
        if (connected() && in_sending_control_mode()) {
            if (_senderInVehicle && _receiverIp != msg.operator_operator_ip_address) {
                _receiverIp = msg.operator_operator_ip_address;
                ROS_INFO("%s: sending to operator ip address %s",
                         ros::this_node::getName().c_str(), _receiverIp.c_str());
                for (auto rosMsgSender : _rosMsgSenders)
                    rosMsgSender->udpSender->change_destination(_receiverIp);
            } else if (!_senderInVehicle && _receiverIp != msg.vehicle_vehicle_ip_address) {
                _receiverIp = msg.vehicle_vehicle_ip_address;
                ROS_INFO("%s: sending to vehicle ip address %s",
                         ros::this_node::getName().c_str(), _receiverIp.c_str());
                for (auto rosMsgSender : _rosMsgSenders)
                    rosMsgSender->udpSender->change_destination(_receiverIp);
            }
        }
    }

    void sendMessageReceived(const ros::MessageEvent<T const>& event, std::shared_ptr<RosMsgSender> rosMsgSender) {
        if (connected() && in_sending_control_mode()) {
            int nofBytesSent = rosMsgSender->udpSender->send_ros_msg(*event.getMessage());
            if (!rosMsgSender->printedInfo) {
                ROS_INFO("%s: sending topic %s - msg size %d", ros::this_node::getName().c_str(),
                         rosMsgSender->sendMsgSubs.getTopic().c_str(), nofBytesSent);
                rosMsgSender->printedInfo = true;
            }
        }
    }

    bool in_sending_control_mode(){
        if (_sendingControlModes.size() == 0)
            return true;

        return std::any_of(_sendingControlModes.begin(), _sendingControlModes.end(),
            [this](uint8_t sendingMode){ return _controlMode == sendingMode; });
    }
};
}; // namespace tod_network
