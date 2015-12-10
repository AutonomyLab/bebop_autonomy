# Ardrone3GPSStateHomeTypeAvailabilityChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Availability of the return home types in a map : for each type other args will be sent by the drone

Header header

# The type of the return home
uint8 type_TAKEOFF=0  # The drone will try to return to the take off position
uint8 type_PILOT=1  # The drone will try to return to the pilot position
uint8 type_FIRST_FIX=2  # The drone has not enough information, it will try to return to the first GPS fix
uint8 type
# 1 if this type is available, 0 otherwise
uint8 available
