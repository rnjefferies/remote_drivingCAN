// Copyright 2021 Schimpe based on Oliver Hamann's ffcfstress
#pragma once
#include <linux/input.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include "bitmaskros.h"

#define DEFAULT_AXIS_INDEX          0
#define DEFAULT_AXIS_CODE       ABS_X

class UsbEventDevice {
public:
    explicit UsbEventDevice(const std::string &deviceNamespace);
    ~UsbEventDevice() { if (ok()) delete_effect(); }
    bool ok() const { return _ok; }
    void set_force_feedback(const double ffValue);
    void reset();

private:
    bool _ok{false};
    const std::vector<std::string> axis_names{ "X", "Y", "Z", "RX", "RY", "RZ", "WHEEL" };
    const std::vector<int> axis_codes{ ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_WHEEL };

    int _axis_index{DEFAULT_AXIS_INDEX};
    int _axis_code{DEFAULT_AXIS_CODE};
    bool _autocenter_off;
    int _device_handle;
    int _axis_min, _axis_max;
    struct ff_effect _effect;

    bool open_device(const std::string &deviceNamespace);
    void initialize_device();
    void create_event(const double force);
    void delete_effect();
};
