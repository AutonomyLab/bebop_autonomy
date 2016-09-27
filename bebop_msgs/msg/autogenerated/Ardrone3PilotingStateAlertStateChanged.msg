# Ardrone3PilotingStateAlertStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Alert state.

Header header

# Drone alert state
uint8 state_none=0  # No alert
uint8 state_user=1  # User emergency alert
uint8 state_cut_out=2  # Cut out alert
uint8 state_critical_battery=3  # Critical battery alert
uint8 state_low_battery=4  # Low battery alert
uint8 state_too_much_angle=5  # The angle of the drone is too high
uint8 state
