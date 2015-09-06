# Ardrone3PilotingStateAlertStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-09-05
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Drone alert state changed

Header header

# Drone alert state
uint8 state_none=0  # No alert
uint8 state_user=1  # User emergency alert
uint8 state_cut_out=2  # Cut out alert
uint8 state_critical_battery=3  # Critical battery alert
uint8 state_low_battery=4  # Low battery alert
uint8 state_too_much_angle=5  # The angle of the drone is too high
uint8 state
