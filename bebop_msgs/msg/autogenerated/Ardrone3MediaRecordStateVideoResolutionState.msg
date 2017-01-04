# Ardrone3MediaRecordStateVideoResolutionState
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Video resolution.\n Informs about streaming and recording video resolutions.\n Note that this is only an indication about what the resolution should be. To know the real resolution, you should get it from the frame.

Header header

# Streaming resolution
uint8 streaming_res360p=0  # 360p resolution.
uint8 streaming_res480p=1  # 480p resolution.
uint8 streaming_res720p=2  # 720p resolution.
uint8 streaming_res1080p=3  # 1080p resolution.
uint8 streaming
# Recording resolution
uint8 recording_res360p=0  # 360p resolution.
uint8 recording_res480p=1  # 480p resolution.
uint8 recording_res720p=2  # 720p resolution.
uint8 recording_res1080p=3  # 1080p resolution.
uint8 recording
