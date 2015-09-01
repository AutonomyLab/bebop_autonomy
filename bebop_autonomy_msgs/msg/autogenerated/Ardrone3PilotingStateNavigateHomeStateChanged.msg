# Ardrone3PilotingStateNavigateHomeStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-08-31 18:59:03.777083
# Generator: generate.py @ f74e92a
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Navigating home state

Header header

# State of navigate home
uint8 state_available=0
uint8 state_inProgress=1
uint8 state_unavailable=2
uint8 state_pending=3
uint8 state
# Reason of the state
uint8 reason_userRequest=0
uint8 reason_connectionLost=1
uint8 reason_lowBattery=2
uint8 reason_finished=3
uint8 reason_stopped=4
uint8 reason_disabled=5
uint8 reason_enabled=6
uint8 reason
