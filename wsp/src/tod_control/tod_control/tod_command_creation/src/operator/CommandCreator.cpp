// Copyright 2020 Simon Hoffmann
#include "CommandCreator.h"

CommandCreator::CommandCreator(ros::NodeHandle& nodeHandle) : _nh(nodeHandle) {
    _joystickSubs = _nh.subscribe("/Operator/InputDevices/joystick", 1,
                                         &CommandCreator::callback_joystick_msg, this);
    _statusSubs = _nh.subscribe("/Operator/Manager/status_msg", 1,
                                       &CommandCreator::callback_status_msg, this);
    _primaryControlPub = _nh.advertise<tod_msgs::PrimaryControlCmd>("primary_control_cmd", 1);
    _secondaryControlPub = _nh.advertise<tod_msgs::SecondaryControlCmd>("secondary_control_cmd", 1);
    _vehParams = std::make_unique<tod_core::VehicleParameters>(_nh);

    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INCREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DECREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_LEFT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_RIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::FLASHLIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::FRONTLIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::HONK, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INCREASE_GEAR, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DECREASE_GEAR, 0));


    if (!_nh.getParam(ros::this_node::getName() + "/ConstraintSteeringRate", _constraintSteeringRate))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /ConstraintSteeringRate - using "
                                                   << (_constraintSteeringRate ? "true" : "false"));

    if (!_nh.getParam(ros::this_node::getName() + "/InvertSteeringInGearReverse", _invertSteeringInGearReverse))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /InvertSteeringInGearReverse - using "
                                                   << (_invertSteeringInGearReverse ? "true" : "false"));

    if (!_nh.getParam(ros::this_node::getName() + "/maxVelocity", _maxSpeedms))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxVelocity - using "
                                                   << _maxSpeedms << " m/s");

    if (!_nh.getParam(ros::this_node::getName() + "/maxAcceleration", _maxAcceleration))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxAcceleration - using "
                                                   << _maxAcceleration << " m/s^2");

    if (!_nh.getParam(ros::this_node::getName() + "/maxDeceleration", _maxDeceleration))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxDeceleration - using "
                                                   << _maxDeceleration << " m/s^2");

    if (!_nh.getParam(ros::this_node::getName() + "/maxSteeringWheelAngleRate", _maxSteeringWheelAngleRate))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /maxSteeringWheelAngleRate - using "
                                                   << _maxSteeringWheelAngleRate << " rad/s");
}

void CommandCreator::run() {
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        if (_vehParams->vehicle_id_has_changed()) {
            _vehParams->load_parameters();
        }
        if (_joystickInputSet && _status == tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
            _primaryControlMsg.header.stamp = ros::Time::now();
            _secondaryControlMsg.header.stamp = ros::Time::now();
            _primaryControlPub.publish(_primaryControlMsg);
            _secondaryControlPub.publish(_secondaryControlMsg);
        }
        _joystickInputSet = false;
        loop_rate.sleep();
    }
}

void CommandCreator::callback_joystick_msg(const sensor_msgs::Joy::ConstPtr& msg) {
    if (_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
        calculate_steering_wheel_angle(_primaryControlMsg, msg->axes);
        calculate_desired_velocity(_primaryControlMsg, msg, _secondaryControlMsg.gearPosition);
        set_gear(_secondaryControlMsg, msg->buttons, _primaryControlMsg.velocity);
        set_indicator(_secondaryControlMsg, msg->buttons);
        set_light(_secondaryControlMsg, msg->buttons);
        set_honk(_secondaryControlMsg, msg->buttons);
        _joystickInputSet = true;
    }
}

void CommandCreator::calculate_steering_wheel_angle(tod_msgs::PrimaryControlCmd& out,
        const std::vector<float>& axes) {
    static ros::Time tPrev;
    static ros::Duration dur;
    static double oldSetSWA{0.0};
    // calc desired SWA
    double newDesiredSWA = axes.at(joystick::AxesPos::STEERING) * _vehParams->get_max_swa_rad();

    if (_constraintSteeringRate) { // constraint steering rate
        dur = ros::Time::now() - tPrev;
        double newSetSWA = std::min(newDesiredSWA, oldSetSWA + dur.toSec() * _maxSteeringWheelAngleRate);
        newSetSWA = std::max(newSetSWA, oldSetSWA - dur.toSec() * _maxSteeringWheelAngleRate);
        oldSetSWA = newSetSWA;
        out.steeringWheelAngle = newSetSWA;
        tPrev = ros::Time::now();
    } else { // output unconstraint SWA
        out.steeringWheelAngle = newDesiredSWA;
    }

    if (_invertSteeringInGearReverse &&
        _secondaryControlMsg.gearPosition == eGearPosition::GEARPOSITION_REVERSE)
        out.steeringWheelAngle = -out.steeringWheelAngle;
}

void CommandCreator::calculate_desired_velocity(tod_msgs::PrimaryControlCmd &out,
        const sensor_msgs::Joy::ConstPtr &msg, const int gear) {
    if (gear == eGearPosition::GEARPOSITION_PARK || gear == eGearPosition::GEARPOSITION_NEUTRAL) {
        out.velocity = 0.0;
        return;
    }

    static ros::Time prevTime = ros::Time::now();
    float a_soll = 0;
    float changeOperator;

    // read Param Server
    bool inputDeviceHasSeparateBrakingAxis{true};
    if (!ros::param::get("/Operator/InputDevices/InputDevice/InputDeviceHasSeparateBrakingAxis/",
                         inputDeviceHasSeparateBrakingAxis)) {
        ROS_ERROR_ONCE("'/Operator/InputDevices/InputDevice/InputDeviceHasSeparateBrakingAxis/' was not set. "
                       "StandardMode is used!");
    }
    // evaluate Speed Change by operator
    if (inputDeviceHasSeparateBrakingAxis) {
        changeOperator = (msg->axes.at(joystick::AxesPos::THROTTLE) - msg->axes.at(joystick::AxesPos::BRAKE)) / 2.0;
    } else {
        changeOperator = msg->axes.at(joystick::AxesPos::THROTTLE);
    }

    //Calculate Acceleration demand by operator
    static float deadzoneThrottle{0.05}, deadzoneBrake{0.05};
    if (changeOperator >= 0) {
        a_soll = _maxAcceleration * (std::max(changeOperator, (float) deadzoneThrottle) - deadzoneThrottle); //acc
    } else {
        a_soll = _maxDeceleration * (std::min(changeOperator, (float) -deadzoneBrake) + deadzoneBrake); // decelerate
    }

    //Integrate Speed
    ros::Duration dt = ros::Time::now() - prevTime;
    prevTime = ros::Time::now();
    out.velocity = out.velocity + dt.toSec() * a_soll;

    // Handle Speed Button Increase/Decrease
    if (msg->buttons.at(joystick::ButtonPos::INCREASE_SPEED) == 1
        && _prevButtonState.at(joystick::ButtonPos::INCREASE_SPEED) == 0) {
        out.velocity += 1.0 / 3.6; // kmh increments
    }
    if (msg->buttons.at(joystick::ButtonPos::DECREASE_SPEED) == 1
        && _prevButtonState.at(joystick::ButtonPos::DECREASE_SPEED) == 0) {
        out.velocity -= 1.0 / 3.6; // kmh increments
    }
    _prevButtonState.at(joystick::ButtonPos::INCREASE_SPEED) = msg->buttons.at(joystick::ButtonPos::INCREASE_SPEED);
    _prevButtonState.at(joystick::ButtonPos::DECREASE_SPEED) = msg->buttons.at(joystick::ButtonPos::DECREASE_SPEED);

    // Saturate Speed integration and limit Acceleration
    if (out.velocity > _maxSpeedms) {
        out.velocity = _maxSpeedms; //Limit demanded speed
    } else if (out.velocity < 0) {
        out.velocity = 0; //Limit demanded speed to zero
    }
    out.acceleration = a_soll;
}

void CommandCreator::callback_status_msg(const tod_msgs::Status &msg) {
    if (_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION
        && msg.tod_status != tod_msgs::Status::TOD_STATUS_TELEOPERATION) {
        init_control_messages();
    }
    _status = msg.tod_status;
}

void CommandCreator::set_gear(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) {
    static int maxGear{3};
    static int minGear{1};

    if (currentVelocity >= 0.01)
        return;

    // Increase Gear
    if (buttonState.at(joystick::ButtonPos::INCREASE_GEAR) == 1
        && _prevButtonState.at(joystick::ButtonPos::INCREASE_GEAR) == 0) {
        if (out.gearPosition < maxGear)
            out.gearPosition += 1;
    }
    // Decrease Gear
    if (buttonState.at(joystick::ButtonPos::DECREASE_GEAR) == 1
        && _prevButtonState.at(joystick::ButtonPos::DECREASE_GEAR) == 0) {
        if (out.gearPosition > minGear)
            out.gearPosition -= 1;
    }
    _prevButtonState.at(joystick::ButtonPos::INCREASE_GEAR) = buttonState.at(joystick::ButtonPos::INCREASE_GEAR);
    _prevButtonState.at(joystick::ButtonPos::DECREASE_GEAR) = buttonState.at(joystick::ButtonPos::DECREASE_GEAR);
}

void CommandCreator::set_indicator(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState) {
    if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 1
        && buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 0) {
        out.indicator = 1; // Indicator Left

    } else if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 0
               && buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 1) {
        out.indicator = 2; // Indicator Right

    } else if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 1
               && buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 1) {
        out.indicator = 3; // Indicator Both

    } else {
        out.indicator = 0; // Indicator Off
    }
}

void CommandCreator::set_light(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState) {
    // flashLight
    out.flashLight = buttonState.at(joystick::ButtonPos::FLASHLIGHT);

    // headLight
    if (buttonState.at(joystick::ButtonPos::FRONTLIGHT) == 1
        && _prevButtonState.at(joystick::ButtonPos::FRONTLIGHT) == 0) {
        if (out.headLight == 0) {
            out.headLight = 1;
        } else {
            out.headLight = 0;
        }
    }
    _prevButtonState.at(joystick::ButtonPos::FRONTLIGHT) = buttonState.at(joystick::ButtonPos::FRONTLIGHT);
}

void CommandCreator::set_honk(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState) {
    out.honk = (bool) buttonState.at(joystick::ButtonPos::HONK);
}

void CommandCreator::init_control_messages() {
    _primaryControlMsg.acceleration = 0;
    _primaryControlMsg.steeringWheelAngle = 0;
    _primaryControlMsg.velocity = 0;
    _secondaryControlMsg.gearPosition = 1;
    _secondaryControlMsg.headLight = 0;
    _secondaryControlMsg.honk = 0;
    _secondaryControlMsg.indicator = 0;
    _secondaryControlMsg.wiper = 0;
}
