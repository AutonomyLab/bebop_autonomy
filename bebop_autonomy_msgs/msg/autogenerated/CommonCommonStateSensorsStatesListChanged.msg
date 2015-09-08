# CommonCommonStateSensorsStatesListChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-09-07
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Sensors states list

Header header

# Sensor name
uint8 sensorName_IMU=0  # Inertial Measurement Unit sensor
uint8 sensorName_barometer=1  # Barometer sensor
uint8 sensorName_ultrasound=2  # Ultrasonic sensor
uint8 sensorName_GPS=3  # GPS sensor
uint8 sensorName_magnetometer=4  # Magnetometer sensor
uint8 sensorName_vertical_camera=5  # Vertical Camera sensor
uint8 sensorName
# Sensor state (1 if the sensor is OK, 0 if the sensor is NOT OK)
uint8 sensorState
