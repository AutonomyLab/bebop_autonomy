# Ardrone3MediaRecordStateVideoStateChangedV2
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Video record state.

Header header

# State of device video recording
uint8 state_stopped=0  # Video is stopped
uint8 state_started=1  # Video is started
uint8 state_notAvailable=2  # The video recording is not available
uint8 state
# Error to explain the state
uint8 error_ok=0  # No Error
uint8 error_unknown=1  # Unknown generic error
uint8 error_camera_ko=2  # Video camera is out of order
uint8 error_memoryFull=3  # Memory full ; cannot save one additional video
uint8 error_lowBattery=4  # Battery is too low to start/keep recording.
uint8 error
