# tod_network

This package contains classes for serializing, sending and receiving ROS messages and configs. Transmission can either be done via UDP or TCP (using MQTT).

## Connection configs

In the file `connection_configs.h`, the respective ports on vehicle and operator side
can be modified.

## Sender and receiver nodes

`tod_sender.h` has a boolean called `senderInVehicle`. This decides about the
topic name, respectively `/Vehicle/…` or `/Operator/…`.

## Dependencies

* ROS Packages: see `package.xml`
* [PahoMqttCpp](https://github.com/eclipse/paho.mqtt.cpp)
  ```bash
  # paho.mqtt.c (required by paho.mqtt.cpp)
  cd /tmp && git clone https://github.com/eclipse/paho.mqtt.c.git
  cd paho.mqtt.c/ && git checkout v1.3.1
  sudo cmake -Bbuild -H. -DPAHO_WITH_SSL=ON -DPAHO_ENABLE_TESTING=OFF
  sudo cmake --build build/ --target install && sudo ldconfig
  # paho.mqtt.cpp
  cd /tmp && git clone https://github.com/eclipse/paho.mqtt.cpp.git
  cd paho.mqtt.cpp/ && git checkout v1.1
  sudo cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=FALSE -DPAHO_BUILD_SAMPLES=TRUE
  sudo cmake --build build/ --target install && sudo ldconfig
  ```
