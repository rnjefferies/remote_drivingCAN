// Copyright 2021 TUMFTM
#include "Interface.h"

Interface::Interface(int argc, char **argv) : Application(argc, argv) {
    ros::NodeHandle nh = _ros->get_node_handle();
    _vehParams = std::make_unique<tod_core::VehicleParameters>(nh);
    _camParams = std::make_unique<tod_core::CameraParameters>(nh);
    _lidarParams = std::make_unique<tod_core::LidarParameters>(nh);
    _transformParams = std::make_unique<tod_core::TransformParameters>(nh);

    CreateScene();
}

void Interface::CreateScene() {
    _activeScene = std::make_shared<Scene>();

    std::map<std::string, Entity> coordinateSystems;
    CreateCosysEntities(coordinateSystems);
    CreateCameraFramebufferAndTopViewEntities(coordinateSystems);
    CreateSafeCorridorControlEntities(coordinateSystems);
    CreateSharedControlEntities(coordinateSystems);
    CreateDisplayEntities(coordinateSystems);
    CreateVehicleModelEntities(coordinateSystems);
    CreateGridAndFloorEntities(coordinateSystems);
    CreateVideoEntities(coordinateSystems);
    CreateLaneEntities(coordinateSystems);
    CreateLaserScanEntities(coordinateSystems);

    bool couldDeserialize = DeserializeEntityData();
    CreateVideoMeshes(couldDeserialize);

    // check that all initialized entities have a parent except for ftm entity
    auto view = _activeScene->_registry.view<TransformComponent>();
    for (auto entity : view) {
        TransformComponent& myTf = _activeScene->_registry.get<TransformComponent>(entity);
        const TagComponent& myTag = _activeScene->_registry.get<TagComponent>(entity);
        if (myTf.ParentEntity.GetHandle() == entt::null) {
            if (myTag.Tag != "Grid" && myTag.Tag != "ftm" && myTag.Tag != "MainFramebuffer") {
                myTf.setParent(coordinateSystems.at("base_footprint"));
                const auto& parentTag =
                    _activeScene->_registry.get<TagComponent>(myTf.ParentEntity.GetHandle());
            }
        }
    }
}

void Interface::CreateCosysEntities(std::map<std::string, Entity> &coordinateSystems) {
    Entity ftm = TodStandardEntities::CoordinateSystem::create(_activeScene, "ftm");
    coordinateSystems.insert_or_assign(ftm.GetComponent<TagComponent>().Tag, ftm);

    Entity bf = TodStandardEntities::CoordinateSystem::create(_activeScene, "base_footprint");
    DynamicDataComponent& dyn = bf.AddComponent<DynamicDataComponent>();
    dyn.HasNewData = true; // has new data for first render loop iteration
    _activeScene->setBaseFootPrint(bf.GetHandle());
    _ros->addSubscriber<nav_msgs::Odometry>(
        "/Operator/VehicleBridge/odometry", TodStandardEntities::BaseFootprint::onPositionUpdate, bf);
    coordinateSystems.insert_or_assign(bf.GetComponent<TagComponent>().Tag, bf);
    bf.GetComponent<TransformComponent>().setParent(ftm);

    std::string parentTag = bf.GetComponent<TagComponent>().Tag;
    for (const auto& tf : _transformParams->get_transforms()) {
        geometry_msgs::TransformStamped tf2parent = _ros->tfLookup(parentTag, tf.child_frame_id);
        const std::string& child = tf2parent.child_frame_id;
        Entity newCosys = TodStandardEntities::CoordinateSystem::create(_activeScene, "Cosys" + child);
        auto &tfCmp = newCosys.GetComponent<TransformComponent>();
        tfCmp.setTranslation(glm::vec3(tf2parent.transform.translation.x,
                                       tf2parent.transform.translation.y,
                                       tf2parent.transform.translation.z));
        double y, p, r;
        tf2::getEulerYPR(tf2parent.transform.rotation, y, p, r);
        tfCmp.setRotation(glm::vec3(r, p, y));
        newCosys.GetComponent<TransformComponent>().setParent(bf);
        newCosys.GetComponent<RenderableElementComponent>().StaticShow = false;
        coordinateSystems.insert_or_assign(child, newCosys);
    }
}

void Interface::CreateDisplayEntities(const std::map<std::string, Entity> &coordinateSystems) {
    Entity velocityDisplay = TodStandardEntities::Display::create(
        _activeScene, "Velocity Display", coordinateSystems.at("base_footprint"), RosInterface::getPackagePath());
    float displayHeight{ 0.20f };
    float displayX{ 1.75f };
    velocityDisplay.GetComponent<TransformComponent>().setTranslation(glm::vec3(displayX, -0.45f, displayHeight));
    _ros->addSubscriber<tod_msgs::VehicleData>("/Operator/VehicleBridge/vehicle_data",
                                               TodStandardEntities::Display::onSpeedUpdate, velocityDisplay);

    Entity desiredVelocityDisplay = TodStandardEntities::Display::create(
        _activeScene, "Desired Velocity Display", coordinateSystems.at("base_footprint"),
        RosInterface::getPackagePath());
    desiredVelocityDisplay.GetComponent<TransformComponent>().setTranslation(glm::vec3(displayX, 0.0f, displayHeight));
    _ros->addSubscriber<tod_msgs::PrimaryControlCmd>(
        "primary_control_cmd", TodStandardEntities::Display::onDesiredSpeedUpdate, desiredVelocityDisplay);

    Entity gearDisplay = TodStandardEntities::Display::create(
        _activeScene, "Gear Display", coordinateSystems.at("base_footprint"), RosInterface::getPackagePath());
    gearDisplay.GetComponent<TransformComponent>().setTranslation(glm::vec3(displayX, 0.3f, displayHeight));
    _ros->addSubscriber<tod_msgs::VehicleData>("/Operator/VehicleBridge/vehicle_data",
                                               TodStandardEntities::Display::onGearUpdate, gearDisplay);

    Entity desiredGearDisplay = TodStandardEntities::Display::create(
        _activeScene, "Desired Gear Display", coordinateSystems.at("base_footprint"), RosInterface::getPackagePath());
    desiredGearDisplay.GetComponent<TransformComponent>().setTranslation(glm::vec3(displayX, 0.5f, displayHeight));
    _ros->addSubscriber<tod_msgs::SecondaryControlCmd>(
        "secondary_control_cmd", TodStandardEntities::Display::onDesiredGearUpdate, desiredGearDisplay);
}

void Interface::CreateVehicleModelEntities(const std::map<std::string, Entity> &coordinateSystems) {
    std::string modelPath = "/vehicle_config/" + _vehParams->get_vehicle_id() + "/model-mesh/";
    double zScaling = 0.1;
    tofGL::ModelLoader *loader = new tofGL::ModelLoader(_activeScene,
                                                        RosInterface::getPackagePath("tod_vehicle_config")
                                                            + modelPath);
    Entity vehicleModel = loader->loadModel("model_chassis", coordinateSystems.at("base_footprint"),
                                            glm::vec3(1.0f, 1.0f, zScaling),
                                            glm::vec3(glm::radians(0.0f), 0.0f, 0.0f),
                                            RosInterface::getPackagePath());
    Entity steeringWheel = loader->loadModel("model_steeringWheel", coordinateSystems.at("base_footprint"),
                                             glm::vec3(1.0f, 1.0f, 1.0f),
                                             glm::vec3(0.0f, 0.0f , 0.0f),
                                             RosInterface::getPackagePath());
    Entity wheelFrontLeft = loader->loadModel("model_wheel", coordinateSystems.at("base_footprint"),
                                              glm::vec3(1.0f, 1.0f, zScaling),
                                              glm::vec3(0.0f, 0.0f, glm::radians(0.0f)),
                                              RosInterface::getPackagePath());
    Entity wheelFrontRight = loader->loadModel("model_wheel", coordinateSystems.at("base_footprint"),
                                               glm::vec3(1.0f, 1.0f, zScaling),
                                               glm::vec3(0.0f, 0.0f, glm::radians(180.0f)),
                                               RosInterface::getPackagePath());
    Entity wheelRearLeft = loader->loadModel("model_wheel", coordinateSystems.at("base_footprint"),
                                             glm::vec3(1.0f, 1.0f, zScaling),
                                             glm::vec3(0.0f, 0.0f, glm::radians(0.0f)),
                                             RosInterface::getPackagePath());
    Entity wheelRearRight = loader->loadModel("model_wheel", coordinateSystems.at("base_footprint"),
                                              glm::vec3(1.0f, 1.0f, zScaling),
                                              glm::vec3(0.0f, 0.0f, glm::radians(180.0f)),
                                              RosInterface::getPackagePath());
    float zPos = 0.1;
    vehicleModel.GetComponent<TransformComponent>().setTranslation(glm::vec3(0.0f, 0.0f, zPos));
    wheelFrontLeft.GetComponent<TransformComponent>().setTranslation(
        glm::vec3(_vehParams->get_distance_front_axle(), _vehParams->get_track_width()/2.0, zPos));
    steeringWheel.GetComponent<TransformComponent>().setTranslation(glm::vec3(0.3f, 0.3f, zPos));
    wheelFrontRight.GetComponent<TransformComponent>().setTranslation(
        glm::vec3(_vehParams->get_distance_front_axle(), -_vehParams->get_track_width()/2.0, zPos));
    wheelRearLeft.GetComponent<TransformComponent>().setTranslation(
        glm::vec3(-_vehParams->get_distance_rear_axle(), _vehParams->get_track_width()/2.0, zPos));
    wheelRearRight.GetComponent<TransformComponent>().setTranslation(
        glm::vec3(-_vehParams->get_distance_rear_axle(), -_vehParams->get_track_width()/2.0, zPos));
}

void Interface::CreateGridAndFloorEntities(const std::map<std::string, Entity> &coordinateSystems) {
    Entity grid = TodStandardEntities::Grid::create(
        _activeScene, "Grid", RosInterface::getPackagePath());
    Entity floor = TodStandardEntities::Floor::create(
        _activeScene, "Floor", RosInterface::getPackagePath(), coordinateSystems.at("base_footprint"));
}

void Interface::CreateCameraFramebufferAndTopViewEntities(const std::map<std::string, Entity> &coordinateSystems) {
    Entity camera = TodStandardEntities::Camera::create(
        _activeScene, "VehicleFollowCamera", coordinateSystems.at("base_footprint"));
    Entity mainFramebuffer = _activeScene->CreateEntity("MainFramebuffer");
    auto& framebuffer = mainFramebuffer.AddComponent<FrameBufferComponent>(true);
    framebuffer.CameraEntity = camera.GetHandle();
    // callback switches camera position to lock backwards (gear rear, InvertSteeringInGearReverse
    // in tod_command_creation.launch)
    _ros->addSubscriber<tod_msgs::VehicleData>("/Operator/VehicleBridge/vehicle_data",
                                               TodStandardEntities::Camera::onGearUpdate, camera);
}

void Interface::CreateVideoEntities(const std::map<std::string, Entity> &coordinateSystems) {
    std::vector<Entity> videos;
    for (const auto& cam : _camParams->get_sensors()) {
        std::string videoName = cam.operator_name;
        videoName.erase(videoName.begin()); // removes '/'
        videos.emplace_back(TodStandardEntities::Video::create(
            _activeScene, videoName, cam.is_fisheye, coordinateSystems.at("base_footprint")));
        std::string imageTopic = "/Operator/Video/" + videoName + "/image_raw";
        _ros->addSubscriber<sensor_msgs::Image>(
            imageTopic, TodStandardEntities::Video::onImageReceived, videos.back());
        std::string projectionTopic = "/Operator/Projection/" + videoName + "/image_raw";
        _ros->addSubscriber<sensor_msgs::Image>(
            projectionTopic, TodStandardEntities::Video::onProjectionReceived, videos.back());
    }
}

void Interface::CreateLaneEntities(const std::map<std::string, Entity> &coordinateSystems) {
    Entity leftLane = TodStandardEntities::Path::create(_activeScene, "Left Front Lane",
                                                        RosInterface::getPackagePath());
    _ros->addSubscriber<nav_msgs::Path>(
        "/Operator/Projection/vehicle_lane_front_left",
        TodStandardEntities::Path::onPathReceived, leftLane, coordinateSystems);
    Entity rightLane = TodStandardEntities::Path::create(_activeScene, "Right Front Lane",
                                                         RosInterface::getPackagePath());
    _ros->addSubscriber<nav_msgs::Path>(
        "/Operator/Projection/vehicle_lane_front_right",
        TodStandardEntities::Path::onPathReceived, rightLane, coordinateSystems);
    Entity rleftLane = TodStandardEntities::Path::create(_activeScene, "Left Rear Lane",
                                                         RosInterface::getPackagePath());
    _ros->addSubscriber<nav_msgs::Path>(
        "/Operator/Projection/vehicle_lane_rear_left",
        TodStandardEntities::Path::onPathReceived, rleftLane, coordinateSystems);
    Entity rrightLane = TodStandardEntities::Path::create(_activeScene, "Right Rear Lane",
                                                          RosInterface::getPackagePath());
    _ros->addSubscriber<nav_msgs::Path>(
        "/Operator/Projection/vehicle_lane_rear_right",
        TodStandardEntities::Path::onPathReceived, rrightLane, coordinateSystems);
}

void Interface::CreateLaserScanEntities(const std::map<std::string, Entity> &coordinateSystems) {
    for (const auto &lidar : _lidarParams->get_sensors()) {
        if (!lidar.is_3D) {
            Entity laserScan = TodStandardEntities::LaserScan::create(
                _activeScene, lidar.name, RosInterface::getPackagePath());
            _ros->addSubscriber<sensor_msgs::LaserScan>(
                "/Operator/Lidar" + lidar.name + "/scan",
                TodStandardEntities::LaserScan::onLaserScanReceived, laserScan, coordinateSystems);
            // some default parent is needed
            laserScan.GetComponent<TransformComponent>().setParent(coordinateSystems.at("base_footprint"));
        }
    }
}

bool Interface::DeserializeEntityData() {
    auto view = _activeScene->_registry.view<TagComponent, VideoComponent>();
    std::vector<Entity> entities;
    for (auto& entityHandle : view)
        entities.emplace_back(entityHandle, _activeScene.get());
    bool ret = Serialization::DeserializeEntities(_ros->getPackagePath(), _camParams->get_vehicle_id(), entities);
    if (!ret) {
        ROS_WARN("%s: First launch of hmi with vehicle id %s.", _ros->getNodeName().c_str(),
                 _camParams->get_vehicle_id().c_str());
    }
    return ret;
}

void Interface::CreateVideoMeshes(const bool couldDeserialize) {
    auto videoHandles = _activeScene->_registry.view<VideoComponent>();
    std::vector<Entity> videoEntities;
    for (auto videoHandle : videoHandles)
        videoEntities.emplace_back(Entity(videoHandle, _activeScene.get()));
    std::vector<std::future<void>> futures;
    for (auto& videoEntity : videoEntities) {
        auto &video = videoEntity.GetComponent<VideoComponent>();
        auto &transform = videoEntity.GetComponent<TransformComponent>();
        geometry_msgs::TransformStamped tf2cam = _ros->tfLookup(video.CameraName, "base_footprint");
        if (!couldDeserialize) {
            // if data could not be deserialized, i.e., first launch with this vehicle,
            // move video streams next to each other
            if (!video.IsFisheye) {
                static int shiftNonFisheye{0};
                transform.setTranslation(glm::vec3(5.0f, 1.5f - float(shiftNonFisheye++), 0.0f));
            } else {
                static int shiftFisheye{0};
                transform.setTranslation(glm::vec3(5.0f, 1.5f - float(shiftFisheye++), 1.0f));
            }
        }
        // initialize mesh depending on camera type
        if (!video.IsFisheye) {
            PinholeModel camMdl(video.CameraName, _camParams->get_vehicle_id());
            _ros->getParam(_ros->getNodeName() + "/" + video.CameraName + "/image_width", video.WidthRaw);
            _ros->getParam(_ros->getNodeName() + "/" + video.CameraName + "/image_height", video.HeightRaw);
            futures.emplace_back(
                std::async(std::launch::async,
                           TodStandardEntities::Video::init_mesh_from_pinhole_model,
                           std::ref(videoEntity), camMdl, tf2cam));
        } else {
            OcamModel camMdl(video.CameraName, _camParams->get_vehicle_id());
            video.WidthRaw = camMdl.width_raw;
            video.HeightRaw = camMdl.height_raw;
            futures.emplace_back(
                std::async(std::launch::async,
                           TodStandardEntities::Video::init_mesh_from_ocam_model,
                           std::ref(videoEntity), camMdl, tf2cam));
        }
    }
}

void Interface::CreateSafeCorridorControlEntities(const std::map<std::string, Entity> &coordinateSystems) {
    Entity freeCorridor = TodStandardEntities::Polygon::create(
        _activeScene, "FreeCorridor", RosInterface::getPackagePath());
    _ros->addSubscriber<tod_msgs::ColoredPolygon>(
        "/Operator/SafeCorridorControl/corridor",
        TodStandardEntities::Polygon::onColoredPolygonReceived, freeCorridor, coordinateSystems);
}

void Interface::CreateSharedControlEntities(const std::map<std::string, Entity> &coordinateSystems) {
    Entity sharedControlObjects = TodStandardEntities::ObjectList::create(
        _activeScene, "SharedControlObjects", RosInterface::getPackagePath());
    _ros->addSubscriber<tod_msgs::ObjectList>(
        "/Operator/SharedControl/avoided_obstacles",
        TodStandardEntities::ObjectList::processAndStoreMsgIntoObjectListEntity,
        sharedControlObjects, coordinateSystems);

    Entity predictedPolygon = TodStandardEntities::Polygon::create(
        _activeScene, "SharedControlPredictedPolygon", RosInterface::getPackagePath());
    _ros->addSubscriber<tod_msgs::ColoredPolygon>(
        "/Operator/SharedControl/predicted_polygon",
        TodStandardEntities::Polygon::onColoredPolygonReceived, predictedPolygon, coordinateSystems);
    predictedPolygon.GetComponent<TransformComponent>().setTranslation(glm::vec3(0.0f, 0.0f, 0.055f));

    Entity raceTrack = TodStandardEntities::Polygon::create(
        _activeScene, "SharedControlRaceTrack", RosInterface::getPackagePath());
    _ros->addSubscriber<tod_msgs::ColoredPolygon>(
        "/Operator/SharedControl/race_track",
        TodStandardEntities::Polygon::onColoredPolygonReceived, raceTrack, coordinateSystems);
}
