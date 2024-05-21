#include <ros/ros.h>
#include <tod_msgs/PrimaryControlCmd.h>
#include <tod_msgs/SecondaryControlCmd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h> 
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <algorithm>

int s; // Socket descriptor for CAN communication.
struct sockaddr_can addr;
struct ifreq ifr;
tod_msgs::SecondaryControlCmd lastSecondaryCmd; // Stores the last gear position for use in CAN message.

// Initialize SocketCAN for communication.
void initSocketCAN(const std::string& interface) {
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return;
    }

    strcpy(ifr.ifr_name, interface.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return;
    }
}

// Send a CAN frame.
void sendCANFrame(const can_frame& frame) {
    int nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "Failed to send frame" << std::endl;
    }
}


int mapAccelerationToBrakingValue(float acceleration) {
    const float minAcceleration = -8.55f; // Minimum input from the device (strongest braking).
    const float maxAcceleration = 0.0f; // No braking.
    const int minBrakingValue = 0; // Corresponds to no braking in CAN message.
    const int maxBrakingValue = 1024; // Corresponds to maximum braking in CAN message.

    // Invert mapping so that stronger negative inputs map to higher CAN values.
    if (acceleration >= 0) {
        return 0; // No braking for non-negative acceleration values.
    }

    // Scale and invert the acceleration value to the braking range.
    // More negative acceleration should result in a higher braking value.
    float proportion = (acceleration - maxAcceleration) / (minAcceleration - maxAcceleration);
    int brakingValue = static_cast<int>(proportion * (maxBrakingValue - minBrakingValue) + minBrakingValue);

    // Ensure brakingValue is within the expected range.
    return std::clamp(brakingValue, minBrakingValue, maxBrakingValue);
}

// Scale and invert steering input according to CAN specifications.
float scaleInputToTarget(float input_value, float max_input_value, float max_target_value) {
    return (input_value / max_input_value) * max_target_value;
}

// Adjusts steering angle for input inversion and CAN range.
float scaleAndInvertSteering(float input_angle, float input_min, float input_max, float target_min, float target_max) {
    float normalized = (input_angle - input_min) / (input_max - input_min);
    float scaled_inverted = (1.0f - normalized) * (target_max - target_min) + target_min;
    return scaled_inverted;
}

// Prepares and sends a CAN message with the control commands.
void sendControlCommandToCan(const tod_msgs::PrimaryControlCmd::ConstPtr& cmd) {
    can_frame frame;
    frame.can_id = 0x183;
    frame.can_dlc = 8;

    // Processing the velocity, braking, and steering commands.
    float scaled_velocity = scaleInputToTarget(cmd->velocity, 10.0f, 600.0f);
    int speed = static_cast<int>(scaled_velocity);
    int brakingValue = mapAccelerationToBrakingValue(cmd->acceleration);
    float adjusted_steering_angle = scaleAndInvertSteering(cmd->steeringWheelAngle, -7.85f, 7.85f, -1024.0f, 1024.0f);
    int steeringValue = static_cast<int>(adjusted_steering_angle) + 1024; // Offset applied for CAN spec.

    // Encoding data into the CAN frame for little-endian.
    frame.data[0] = speed & 0xFF;                  // LSB of speed
    frame.data[1] = (speed >> 8) & 0xFF;           // MSB of speed
    frame.data[2] = brakingValue & 0xFF;           // LSB of braking value
    frame.data[3] = (brakingValue >> 8) & 0xFF;    // MSB of braking value
    frame.data[4] = steeringValue & 0xFF;          // LSB of steering value
    frame.data[5] = (steeringValue >> 8) & 0xFF;   // MSB of steering value

    frame.data[6] = (frame.data[6] & ~0x0C) | (0x2 << 4); // Mask out bits 52 and 53 and set to 0x2

// Adjusting gear position for the CAN message, assuming the gear uses the least significant bits of byte 6.
switch(lastSecondaryCmd.gearPosition) {
    case 1: // Drive
        frame.data[6] = (frame.data[6] & ~0x03) | 0x01; // Set the first two bits to 01
        break;
    case 2: // Neutral
        frame.data[6] = (frame.data[6] & ~0x03) | 0x02; // Set the first two bits to 10
        break;
    case 3: // Reverse
        frame.data[6] = (frame.data[6] & ~0x03) | 0x03; // Set the first two bits to 11
        break;
    default: // Clear the first two bits if not setting
        frame.data[6] &= ~0x03;
        break;
}


    // Setting headlight bit if applicable.
    frame.data[7] = 0x00; // Reset all switch mode bits to 0.

    if (lastSecondaryCmd.headLight == 1) {
        frame.data[7] |= (1 << 4); // Set bit for headlight on.
    }

    // Enabling autonomous driving by default.
    frame.data[7] |= (1 << 7); // Set bit for autonomous driving enabled.

    
    sendCANFrame(frame); // Sending the frame.
}

// ROS callbacks for receiving control commands.
void primaryCmdCallback(const tod_msgs::PrimaryControlCmd::ConstPtr& msg) {
    sendControlCommandToCan(msg);
}

void secondaryCmdCallback(const tod_msgs::SecondaryControlCmd::ConstPtr& msg) {
    lastSecondaryCmd = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ControlCmdToCan");
    ros::NodeHandle nh;

    // Initialize CAN communication and set up subscribers.
    initSocketCAN("vcan0");

    ros::Subscriber primaryCmdSub = nh.subscribe("/Vehicle/VehicleBridge/primary_control_cmd", 10, primaryCmdCallback);
    ros::Subscriber secondaryCmdSub = nh.subscribe("/Vehicle/CommandCreation/secondary_control_cmd", 10, secondaryCmdCallback);

    ros::spin(); // Enter the ROS event loop.

    close(s); // Close the socket before exiting.

    return 0;
}
