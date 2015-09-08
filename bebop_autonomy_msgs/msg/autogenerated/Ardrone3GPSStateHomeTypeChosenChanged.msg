# Ardrone3GPSStateHomeTypeChosenChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-09-07
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: The return home type chosen

Header header

# The type of the return home chosen
uint8 type_TAKEOFF=0  # The drone will try to return to the take off position
uint8 type_PILOT=1  # The drone will try to return to the pilot position
uint8 type_FIRST_FIX=2  # The drone has not enough information, it will try to return to the first GPS fix
uint8 type
