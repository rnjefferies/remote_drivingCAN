// Copyright 2021 TUMFTM
#include <ros/ros.h>
#include <memory>
#include "Interface.h"

int main(int argc, char **argv) {
    auto interface = std::make_unique<Interface>(argc, argv);
    interface->run();
    return 0;
}
