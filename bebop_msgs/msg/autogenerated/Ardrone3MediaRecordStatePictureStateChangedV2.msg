# Ardrone3MediaRecordStatePictureStateChangedV2
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Picture state.

Header header

# State of device picture recording
uint8 state_ready=0  # The picture recording is ready
uint8 state_busy=1  # The picture recording is busy
uint8 state_notAvailable=2  # The picture recording is not available
uint8 state
# Error to explain the state
uint8 error_ok=0  # No Error
uint8 error_unknown=1  # Unknown generic error
uint8 error_camera_ko=2  # Picture camera is out of order
uint8 error_memoryFull=3  # Memory full ; cannot save one additional picture
uint8 error_lowBattery=4  # Battery is too low to start/keep recording.
uint8 error
