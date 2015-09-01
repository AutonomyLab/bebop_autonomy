# Ardrone3MediaRecordStatePictureStateChangedV2
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-08-31 17:44:29.928022
# Generator: generate.py @ 1291966
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: State of device picture recording changed

Header header

# State of device picture recording
uint8 state_ready=0
uint8 state_busy=1
uint8 state_notAvailable=2
uint8 state
# Error to explain the state
uint8 error_ok=0
uint8 error_unknown=1
uint8 error_camera_ko=2
uint8 error_memoryFull=3
uint8 error_lowBattery=4
uint8 error
