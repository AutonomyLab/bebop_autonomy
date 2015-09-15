# Ardrone3MediaRecordStateVideoStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: @deprecated State of video recording

Header header

# State of video
uint8 state_stopped=0  # Video was stopped
uint8 state_started=1  # Video was started
uint8 state_failed=2  # Video was failed
uint8 state_autostopped=3  # Video was auto stopped
uint8 state
# Mass storage id where the video was recorded
uint8 mass_storage_id
