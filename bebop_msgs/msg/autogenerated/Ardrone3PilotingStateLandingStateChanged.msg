# Ardrone3PilotingStateLandingStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Landing state.\n Only available for fixed wings (which have two landing modes).

Header header

# Drone landing state
uint8 state_linear=0  # Linear landing
uint8 state_spiral=1  # Spiral landing
uint8 state
