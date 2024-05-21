std_msgs/Header header
# see VehicleEnums.h
int8 indicator
int8 gearPosition
int8 honk
int8 wiper
int8 headLight
int8 flashLight
float32 steeringWheelAngle         # steering wheel angle position in rad
float32 curvature                  # current vehicle curvature in 1/m
float32 engineSpeed                # rpm
float32 longitudinalSpeed          # vehicle ACAN data (currently from rear wheel odom)
geometry_msgs/Vector3 linearAcceleration    # m/s^2
