# Ardrone3PilotingStateFlyingStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Flying state.

Header header

# Drone flying state
uint8 state_landed=0  # Landed state
uint8 state_takingoff=1  # Taking off state
uint8 state_hovering=2  # Hovering / Circling (for fixed wings) state
uint8 state_flying=3  # Flying state
uint8 state_landing=4  # Landing state
uint8 state_emergency=5  # Emergency state
uint8 state_usertakeoff=6  # User take off state. Waiting for user action to take off.
uint8 state_motor_ramping=7  # Motor ramping state (for fixed wings).
uint8 state_emergency_landing=8  # Emergency landing state. Drone autopilot has detected defective sensor(s). Only Yaw argument in PCMD is taken into account. All others flying commands are ignored.
uint8 state
