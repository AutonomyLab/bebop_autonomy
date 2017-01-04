# CommonChargerStateMaxChargeRateChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Max charge rate.

Header header

# The current maximum charge rate.
uint8 rate_SLOW=0  # Fully charge the battery at a slow rate. Typically limit max charge current to 512 mA.
uint8 rate_MODERATE=1  # Almost fully-charge the battery at moderate rate (&gt; 512 mA) but slower than the fastest rate.
uint8 rate_FAST=2  # Almost fully-charge the battery at the highest possible rate supported by the charger.
uint8 rate
