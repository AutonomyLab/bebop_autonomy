# Ardrone3PilotingStateGpsLocationChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Drones location changed.\n This event is meant to replace [PositionChanged](#1-4-4).

Header header

# Latitude location in decimal degrees (500.0 if not available)
float64 latitude
# Longitude location in decimal degrees (500.0 if not available)
float64 longitude
# Altitude location in meters.
float64 altitude
# Latitude location error in meters (1 sigma/standard deviation) -1 if not available.
int8 latitude_accuracy
# Longitude location error in meters (1 sigma/standard deviation) -1 if not available.
int8 longitude_accuracy
# Altitude location error in meters (1 sigma/standard deviation) -1 if not available.
int8 altitude_accuracy
