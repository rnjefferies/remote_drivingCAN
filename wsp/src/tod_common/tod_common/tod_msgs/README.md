# tod_msgs

This package contains the custom ROS message definitions used throughout the software
stack. The structure of the package can be presented as follows:

- **/msg**: Contains all *.msg files.
- **/srv**: Contains all *.srv files.
- **/include/tod_msgs**: Contains helper header files with the message constants.

## Other

Building the service and messages is done in a typical ROS manner (via CMakeLists.txt and
package.xml).
