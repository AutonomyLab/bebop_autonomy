# CommonFlightPlanStateLockStateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: FlightPlan lock state.\n Represents the fact that the controller is able or not to stop or pause a playing FlightPlan

Header header

# 1 if FlightPlan is locked: cant pause or stop FlightPlan. 0 if FlightPlan is unlocked: pause or stop available.
uint8 LockState
