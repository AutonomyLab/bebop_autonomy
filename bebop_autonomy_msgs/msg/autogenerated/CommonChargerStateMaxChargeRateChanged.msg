# CommonChargerStateMaxChargeRateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-09-02
# Generator: generate.py @ ebb47cc
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: @deprecated The maximum charge rate reported by the firmware.

Header header

# The current maximum charge rate.
uint8 rate_SLOW=0  # Fully charge the battery at a slow rate. Typically limit max charge current to 512 mA.
uint8 rate_MODERATE=1  # Almost fully-charge the battery at moderate rate (&gt; 512 mA) but slower than the fastest rate.
uint8 rate_FAST=2  # Almost fully-charge the battery at the highest possible rate supported by the charger.
uint8 rate
