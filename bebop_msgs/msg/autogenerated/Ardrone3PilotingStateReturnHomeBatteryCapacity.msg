# Ardrone3PilotingStateReturnHomeBatteryCapacity
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Battery capacity status to return home.

Header header

# Status of battery to return home
uint8 status_OK=0  # The battery is full enough to do a return home
uint8 status_WARNING=1  # The battery is about to be too discharged to do a return home
uint8 status_CRITICAL=2  # The battery level is too low to return to the home position
uint8 status_UNKNOWN=3  # Battery capacity to do a return home is unknown. This can be either because the home is unknown or the position of the drone is unknown, or the drone has not enough information to determine how long it takes to fly home.
uint8 status
