# CommonMavlinkStateMavlinkFilePlayingStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-08-31 18:59:03.288589
# Generator: generate.py @ f74e92a
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Playing state of a mavlink flight plan

Header header

# State of the mavlink
uint8 state_playing=0
uint8 state_stopped=1
uint8 state_paused=2
uint8 state
# flight plan file path from the mavlink ftp root
string filepath
# type of the played mavlink file
uint8 type_flightPlan=0
uint8 type_mapMyHouse=1
uint8 type
