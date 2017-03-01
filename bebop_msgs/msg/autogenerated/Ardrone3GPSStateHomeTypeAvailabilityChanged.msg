# Ardrone3GPSStateHomeTypeAvailabilityChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Home type availability.

Header header

# The type of the return home
uint8 type_TAKEOFF=0  # The drone has enough information to return to the take off position
uint8 type_PILOT=1  # The drone has enough information to return to the pilot position
uint8 type_FIRST_FIX=2  # The drone has not enough information, it will return to the first GPS fix
uint8 type_FOLLOWEE=3  # The drone has enough information to return to the target of the current (or last) follow me
uint8 type
# 1 if this type is available, 0 otherwise
uint8 available
