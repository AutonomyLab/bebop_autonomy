# CommonMavlinkStateMavlinkFilePlayingStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-09-02
# Generator: generate.py @ ebb47cc
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Playing state of a mavlink flight plan

Header header

# State of the mavlink
uint8 state_playing=0  # Mavlink file is playing
uint8 state_stopped=1  # Mavlink file is stopped (arg filepath and type are useless in this state)
uint8 state_paused=2  # Mavlink file is paused
uint8 state
# flight plan file path from the mavlink ftp root
string filepath
# type of the played mavlink file
uint8 type_flightPlan=0  # Mavlink file for FlightPlan
uint8 type_mapMyHouse=1  # Mavlink file for MapMyHouse
uint8 type
