# Ardrone3PilotingStatemoveToChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: The drone moves or moved to a given location.

Header header

# Latitude of the location (in degrees) to reach
float64 latitude
# Longitude of the location (in degrees) to reach
float64 longitude
# Altitude above sea level (in m) to reach
float64 altitude
# Orientation mode of the move to
uint8 orientation_mode_NONE=0  # The drone wont change its orientation
uint8 orientation_mode_TO_TARGET=1  # The drone will make a rotation to look in direction of the given location
uint8 orientation_mode_HEADING_START=2  # The drone will orientate itself to the given heading before moving to the location
uint8 orientation_mode_HEADING_DURING=3  # The drone will orientate itself to the given heading while moving to the location
uint8 orientation_mode
# Heading (relative to the North in degrees). This value is only used if the orientation mode is HEADING_START or HEADING_DURING
float32 heading
# Status of the move to
uint8 status_RUNNING=0  # The drone is actually flying to the given position
uint8 status_DONE=1  # The drone has reached the target
uint8 status_CANCELED=2  # The move to has been canceled, either by a new moveTo command or by a CancelMoveTo command.
uint8 status_ERROR=3  # The move to has not been finished or started because of an error.
uint8 status
