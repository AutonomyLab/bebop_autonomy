# Ardrone3NetworkStateWifiScanListChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-09-03
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: One scanning result found

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
