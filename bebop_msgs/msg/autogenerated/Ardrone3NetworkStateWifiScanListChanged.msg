# Ardrone3NetworkStateWifiScanListChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Wifi scan results.\n Please note that the list is not complete until you receive the event [WifiScanEnded](#1-14-1).

Header header

# SSID of the AP
string ssid
# RSSI of the AP in dbm (negative value)
int16 rssi
# The band : 2.4 GHz or 5 GHz
uint8 band_2_4ghz=0  # 2.4 GHz band
uint8 band_5ghz=1  # 5 GHz band
uint8 band
# Channel of the AP
uint8 channel
