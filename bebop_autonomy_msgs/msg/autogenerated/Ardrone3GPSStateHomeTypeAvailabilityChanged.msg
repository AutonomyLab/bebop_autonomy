# Ardrone3GPSStateHomeTypeAvailabilityChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-08-31 17:44:29.928022
# Generator: generate.py @ 1291966
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Availability of the return home types in a map : for each type other args will be sent by the drone

Header header

# The type of the return home
uint8 type_TAKEOFF=0
uint8 type_PILOT=1
uint8 type_FIRST_FIX=2
uint8 type
# 1 if this type is available, 0 otherwise
uint8 available
