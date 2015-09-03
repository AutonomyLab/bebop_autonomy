# Ardrone3NetworkStateWifiAuthChannelListChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-09-02
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Notify of an Authorized Channel.

Header header

# The band of this channel : 2.4 GHz or 5 GHz
uint8 band_2_4ghz=0  # 2.4 GHz band
uint8 band_5ghz=1  # 5 GHz band
uint8 band
# The authorized channel.
uint8 channel
# Bit 0 is 1 if channel is authorized outside (0 otherwise) ; Bit 1 is 1 if channel is authorized inside (0 otherwise)
uint8 in_or_out
