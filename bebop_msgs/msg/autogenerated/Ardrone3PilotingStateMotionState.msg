# Ardrone3PilotingStateMotionState
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Motion state.\n If [MotionDetection](#1-6-16) is disabled, motion is steady.\n This information is only valid when the drone is not flying.

Header header

# Motion state
uint8 state_steady=0  # Drone is steady
uint8 state_moving=1  # Drone is moving
uint8 state
