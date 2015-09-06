# CommonFlightPlanStateComponentStateListChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-09-05
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
