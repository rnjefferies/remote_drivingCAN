// Copyright 2020 Feiler

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <memory>

#include "IPv4ValidityChecker.h"

class IPv4ValidityCheckerTest : public ::testing::Test {
protected:
    std::unique_ptr<IpAddrValidityChecker> validity_checker;
    void SetUp() override {
        validity_checker.reset(new IPv4ValidityChecker);
    }
};

TEST_F(IPv4ValidityCheckerTest, CheckIpAddrChecker) {
    EXPECT_EQ(validity_checker->validate("127.0.0.1"), true);
    EXPECT_EQ(validity_checker->validate("127.0.0"), false);
    EXPECT_EQ(validity_checker->validate("Dieter"), false);
    EXPECT_EQ(validity_checker->validate("127.0.0.100000"), false);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "IPv4ValidityChecker");
    return RUN_ALL_TESTS();
}
