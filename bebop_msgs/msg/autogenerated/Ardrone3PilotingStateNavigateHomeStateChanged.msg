# Ardrone3PilotingStateNavigateHomeStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Return home state.\n Availability is related to gps fix, magnetometer calibration.

Header header

# State of navigate home
uint8 state_available=0  # Navigate home is available
uint8 state_inProgress=1  # Navigate home is in progress
uint8 state_unavailable=2  # Navigate home is not available
uint8 state_pending=3  # Navigate home has been received, but its process is pending
uint8 state
# Reason of the state
uint8 reason_userRequest=0  # User requested a navigate home (available-&gt;inProgress)
uint8 reason_connectionLost=1  # Connection between controller and product lost (available-&gt;inProgress)
uint8 reason_lowBattery=2  # Low battery occurred (available-&gt;inProgress)
uint8 reason_finished=3  # Navigate home is finished (inProgress-&gt;available)
uint8 reason_stopped=4  # Navigate home has been stopped (inProgress-&gt;available)
uint8 reason_disabled=5  # Navigate home disabled by product (inProgress-&gt;unavailable or available-&gt;unavailable)
uint8 reason_enabled=6  # Navigate home enabled by product (unavailable-&gt;available)
uint8 reason
