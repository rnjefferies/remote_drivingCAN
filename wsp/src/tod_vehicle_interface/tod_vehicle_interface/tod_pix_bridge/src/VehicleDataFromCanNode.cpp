#include <ros/ros.h>
#include <socketcan_interface/socketcan.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <iomanip>
#include <sys/socket.h>
#include <unistd.h>
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/VehicleEnums.h"
#include <cstring>
#include <iostream>
#include <cmath>

class CANReceiver {
public:
    CANReceiver(const std::string& interface) : interface_(interface) {
        // Attempt to open the socket
        if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            ROS_FATAL("Error while opening socket");
            ros::shutdown();
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, interface_.c_str());
        ioctl(socket_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        // Bind the socket
        if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            ROS_FATAL("Error in socket bind");
            ros::shutdown();
        }

        // ROS Publisher
        vehicle_data_pub = nh_.advertise<tod_msgs::VehicleData>("vehicle_data", 1);
    }

    ~CANReceiver() {
        close(socket_);
    }

    void listen() {
        struct can_frame frame;
        int nbytes;

        while (ros::ok()) {
            nbytes = read(socket_, &frame, sizeof(struct can_frame));
            
            if (nbytes < 0) {
                ROS_ERROR("CAN frame read error");
                continue;
            }

            if (nbytes < sizeof(struct can_frame)) {
                ROS_ERROR("Read incomplete CAN frame");
                continue;
            }

            // Process the frame if it matches the desired ID
            if (frame.can_id == 0x193) { // Adjust as needed for other IDs
                processFrame(frame);
            }
        }
    }

private:
    void processFrame(const struct can_frame &frame) {
        tod_msgs::VehicleData msg;
        msg.header.stamp = ros::Time::now();

       ROS_INFO_STREAM("Received CAN frame with ID: " << std::hex << frame.can_id);
    ROS_INFO_STREAM("Raw data: "
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[0]
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[1]
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[2]
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[3]
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[4]
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[5]
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[6]
        << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[7]);

    // Decode vehicle speed assuming little-endian byte order
int16_t speed_raw = (static_cast<int16_t>(frame.data[1]) << 8) | static_cast<int16_t>(frame.data[0]);
// Debug: Log decoded speed value
ROS_INFO_STREAM("Decoded speed (raw): " << speed_raw);
// Adjust raw speed value from 0.1 km/h unit
msg.longitudinalSpeed = (static_cast<float>(speed_raw) * 0.1f);
// Debug: Log final speed in km/h
ROS_INFO_STREAM("Longitudinal speed (km/h): " << msg.longitudinalSpeed);

   
   // Decode steering angle assuming little-endian byte order
int16_t steering_raw = (static_cast<uint16_t>(frame.data[3]) << 8) | static_cast<uint16_t>(frame.data[2]);
// Adjust by 750 to correct the offset as mentioned
steering_raw -= 750;
ROS_INFO_STREAM("Steering wheel angle (raw): " << steering_raw);
// Convert from degrees to radians
msg.steeringWheelAngle = static_cast<float>(steering_raw) * M_PI / 180.0f;
// Debug: Log final steering wheel angle in radians
ROS_INFO_STREAM("Steering wheel angle (radians): " << msg.steeringWheelAngle);


    // Decode gear position
uint8_t gear_raw = (frame.data[6]) & 0x03;
ROS_INFO_STREAM("Gear position (raw): " << (int)gear_raw);
msg.gearPosition = gear_raw;


vehicle_data_pub.publish(msg);

    }

    ros::NodeHandle nh_;
    ros::Publisher vehicle_data_pub;
    int socket_;
    std::string interface_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "VehicleDataFromCan");

    std::string interface = "can0"; // Change to "vcan0" for testing with a virtual interface
    CANReceiver receiver(interface);
    receiver.listen();

    return 0;
}
