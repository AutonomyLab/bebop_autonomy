# Ardrone3PilotingStatePilotedPOI
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Piloted POI state.

Header header

# Latitude of the location (in degrees) to look at. This information is only valid when the state is pending or running.
float64 latitude
# Longitude of the location (in degrees) to look at. This information is only valid when the state is pending or running.
float64 longitude
# Altitude above sea level (in m) to look at. This information is only valid when the state is pending or running.
float64 altitude
# Status of the move to
uint8 status_UNAVAILABLE=0  # The piloted POI is not available
uint8 status_AVAILABLE=1  # The piloted POI is available
uint8 status_PENDING=2  # Piloted POI has been requested. Waiting to be in state that allow the piloted POI to start
uint8 status_RUNNING=3  # Piloted POI is running
uint8 status
