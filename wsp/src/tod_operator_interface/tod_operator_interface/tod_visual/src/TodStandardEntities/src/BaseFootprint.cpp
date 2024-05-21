// Copyright 2020 TUMFTM
#include "BaseFootprint.h"

namespace TodStandardEntities {

void BaseFootprint::onPositionUpdate(const nav_msgs::OdometryConstPtr& msg, Entity &entity) {
    static ros::Time prevUpdate{ ros::Time::now() };
    static const ros::Duration updatePeriod{ 1.0 / 30.0 }; // update odom at 30 hz only
    if (ros::Time::now() < prevUpdate + updatePeriod)
        return;
    prevUpdate = ros::Time::now();


    auto& dynamic = entity.GetComponent<DynamicDataComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    double r, p, y;
    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(r, p, y);
    auto& transComp = entity.GetComponent<TransformComponent>();
    checkParentFrameConsistency(transComp, msg->header);
    if (msg->pose.pose.position.x > 60000.0f) {
        transComp.setTranslation(
            // lack of floating point precision
            glm::vec3(msg->pose.pose.position.x - 698000.0f, msg->pose.pose.position.y - 5349000.0f, 0.0f));
    } else {
        transComp.setTranslation(glm::vec3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0f));
    }
    transComp.setRotation(glm::vec3(0.0f, 0.0f, y));
    dynamic.HasNewData = true;
}

void BaseFootprint::checkParentFrameConsistency(TransformComponent &transComp, const std_msgs::Header &msgHeader) {
    const std::string& parentTag = transComp.ParentEntity.GetComponent<TagComponent>().Tag;
    if (msgHeader.frame_id != parentTag) {
        ROS_WARN_ONCE("BaseFootprint: odom header frame id (%s) and entity parent tag (%s) do not match",
                      msgHeader.frame_id.c_str(), parentTag.c_str());
    }
}

}; // namespace TodStandardEntities
