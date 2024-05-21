# tod_launch

This package contains high-level launch-files for a convenient startup process of the software stack.
The software stack can be launched with the following commands:

1. Source the workspace:
  `source devel/setup.zsh` or `source devel/setup.bash` depending on your shell
2. Launch the nodes:
    * Operator side:
      `roslaunch tod_launch operator.launch`
    * Vehicle side:
      `roslaunch tod_launch vehicle.launch`
    * Both sides:
      `roslaunch tod_launch both.launch`

## Launch Configuration

The launch files can be configured by the following arguments as required. The arguments are explained in the following:

* `vehicleID`: Identifier (string) of the vehicle that is being teleoperated. Each
vehicle requires the provision of a configuration in the `tod_vehicle_interface/tod_vehicle_config` package and a bridge package
under `tod_vehicle_interface` (for vehicle side only).
* `mode`: Mode (string) that states how the software should be launched. The mode is only relevant for vehicle side.
  * `vehicle`: Used to teleoperate an actual or simulated vehicle. Launches all drivers and
  vehicle hardware interfaces as specified in the bridge package of the respective `vehicleID`.
  * `playbackAll`: Plays back sensor (e.g., camera, lidar, and odometry) and vehicle data signals, coming from a rosbag file.
  * `playbackSim`: Plays back some sensor data (e.g., camera and lidar) streams. In addition, a vehicle simulation node is launched,
  consuming control commands and generating odometry and vehicle data streams.
  * `onlySim`: Launches only the vehicle simulation node.
* `launchPackageName`: Flag (boolean) to enable/disable launch of a package.
Useful if certain packages are not needed, or could not be built due missing
dependencies.
