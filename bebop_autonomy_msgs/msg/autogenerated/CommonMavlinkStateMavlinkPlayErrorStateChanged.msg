# CommonMavlinkStateMavlinkPlayErrorStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-09-02
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: FlightPlan play state error

Header header

# State of play error
uint8 error_none=0  # There is no error
uint8 error_notInOutDoorMode=1  # The drone is not in outdoor mode
uint8 error_gpsNotFixed=2  # The gps is not fixed
uint8 error_notCalibrated=3  # The magnetometer of the drone is not calibrated
uint8 error
