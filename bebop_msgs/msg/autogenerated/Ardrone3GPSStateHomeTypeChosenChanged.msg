# Ardrone3GPSStateHomeTypeChosenChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Home type.\n This choice is made by the drone, according to the [PreferredHomeType](#1-24-4) and the [HomeTypeAvailability](#1-31-1). The drone will choose the type matching with the user preference only if this type is available.

Header header

# The type of the return home chosen
uint8 type_TAKEOFF=0  # The drone will try to return to the take off position
uint8 type_PILOT=1  # The drone will try to return to the pilot position
uint8 type_FIRST_FIX=2  # The drone has not enough information, it will try to return to the first GPS fix
uint8 type
