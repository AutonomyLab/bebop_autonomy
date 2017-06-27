# CommonFlightPlanStateComponentStateListChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: FlightPlan components state list.

Header header

# Drone FlightPlan component id (unique)
uint8 component_GPS=0  # Drone GPS component. State is 0 when the drone needs a GPS fix.
uint8 component_Calibration=1  # Drone Calibration component. State is 0 when the sensors of the drone needs to be calibrated.
uint8 component_Mavlink_File=2  # Mavlink file component. State is 0 when the mavlink file is missing or contains error.
uint8 component_TakeOff=3  # Drone Take off component. State is 0 when the drone cannot take-off.
uint8 component_WaypointsBeyondGeofence=4  # Component for waypoints beyond the geofence. State is 0 when one or more waypoints are beyond the geofence.
uint8 component
# State of the FlightPlan component (1 FlightPlan component OK, otherwise 0)
uint8 State
