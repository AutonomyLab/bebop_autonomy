# CommonCommonStateCountryListKnown
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: List of countries known by the drone.

Header header

# List entry attribute Bitfield. 0x01: First: indicate its the first element of the list. 0x02: Last: indicate its the last element of the list. 0x04: Empty: indicate the list is empty (implies First/Last). All other arguments should be ignored.
uint8 listFlags
# Following of country code with ISO 3166 format, separated by ;. Be careful of the command size allowed by the network used. If necessary, split the list in several commands.
string countryCodes
