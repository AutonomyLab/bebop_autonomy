# CommonFlightPlanStateComponentStateListChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: List of state of drone flightPlan components

Header header

# Drone FlightPlan component id (unique)
uint8 component_GPS=0  # GPS for Drone FlightPlan
uint8 component_Calibration=1  # Calibration for Drone FlightPlan
uint8 component_Mavlink_File=2  # Mavlink file for Drone FlightPlan
uint8 component_TakeOff=3  # Take off
uint8 component
# State of the FlightPlan component (1 FlightPlan component OK, otherwise 0)
uint8 State
