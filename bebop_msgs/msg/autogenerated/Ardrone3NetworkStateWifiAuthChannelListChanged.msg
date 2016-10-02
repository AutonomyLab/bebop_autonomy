# Ardrone3NetworkStateWifiAuthChannelListChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Available wifi channels.\n Please note that the list is not complete until you receive the event [AvailableWifiChannelsCompleted](#1-14-3).

Header header

# The band of this channel : 2.4 GHz or 5 GHz
uint8 band_2_4ghz=0  # 2.4 GHz band
uint8 band_5ghz=1  # 5 GHz band
uint8 band
# The authorized channel.
uint8 channel
# Bit 0 is 1 if channel is authorized outside (0 otherwise) ; Bit 1 is 1 if channel is authorized inside (0 otherwise)
uint8 in_or_out
