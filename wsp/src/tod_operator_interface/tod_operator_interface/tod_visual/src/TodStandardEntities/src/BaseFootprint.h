// Copyright 2020 TUMFTM
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"

namespace TodStandardEntities {

class BaseFootprint {
public:
    static void onPositionUpdate(const nav_msgs::OdometryConstPtr& msg, Entity& entity);

private:
    BaseFootprint() = default;
    static void checkParentFrameConsistency(TransformComponent &transComp, const std_msgs::Header &msgHeader);
};

}; // namespace TodStandardEntities
