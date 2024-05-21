# tod_core

The tod_core package takes care of vehicle specific parameter handling. Depending on the current `vehicleID` in the parameter server, vehicle parameters from package `tod_vehicle_config` are loaded.

## Usage

```C++
#include "tod_core/VehicleParameters.h"

std::unique_ptr<tod_core::VehicleParameters> vehicleParamHandler =
        std::make_unique<tod_core::VehicleParameters>(nh);

while (ros::ok()) {
    ros::spinOnce();

    if (vehicleParamHandler->vehicle_id_has_changed()) {
        vehicleParamHandler->load_parameters();
    }

    std::cout << vehicleParamHandler->get_max_rwa_rad() << std::endl;
}
```

## Dependencies

* see `package.xml`
* yaml-cpp

  ```bash
  sudo apt-get install libyaml-cpp-dev -y
  ```
