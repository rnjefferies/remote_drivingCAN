// Copyright 2020 Feiler
#include "ros/ros.h"
#include "tod_core/TransformParameters.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <memory>

int main(int argc, char **argv) {
    ros::init(argc, argv, "CommonTransformTreePublisherNode");
    ros::NodeHandle nodeHandle;
    auto _transformParams(std::make_unique<tod_core::TransformParameters>(nodeHandle));

    static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    ros::Rate r{10};
    while (ros::ok()) {
        ros::spinOnce();
        if ( _transformParams->vehicle_id_has_changed())
            _transformParams->load_parameters();

        _transformParams->updateStamp();
        tf_broadcaster.sendTransform(_transformParams->get_transforms());
        r.sleep();
    }
    return 0;
}
