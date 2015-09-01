# Ardrone3MediaRecordStateVideoStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-08-31 18:59:03.777083
# Generator: generate.py @ f74e92a
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: @deprecated State of video recording

Header header

# State of video
uint8 state_stopped=0
uint8 state_started=1
uint8 state_failed=2
uint8 state_autostopped=3
uint8 state
# Mass storage id where the video was recorded
uint8 mass_storage_id
