# CommonCommonStateVideoRecordingTimestamp
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Current or last video recording timestamp.\n Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970.\n **Please note that values dont persist after drone reboot**

Header header

# Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970.
uint64 startTimestamp
# Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970. 0 mean that video is still recording.
uint64 stopTimestamp
