# CommonChargerStateCurrentChargeStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-08-31 17:44:29.412157
# Generator: generate.py @ 1291966
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: @deprecated The charge status of the battery changed.

Header header

# Charger status.
uint8 status_DISCHARGING=0
uint8 status_CHARGING_SLOW=1
uint8 status_CHARGING_MODERATE=2
uint8 status_CHARGING_FAST=3
uint8 status_BATTERY_FULL=4
uint8 status
# The current charging phase.
uint8 phase_UNKNOWN=0
uint8 phase_CONSTANT_CURRENT_1=1
uint8 phase_CONSTANT_CURRENT_2=2
uint8 phase_CONSTANT_VOLTAGE=3
uint8 phase_CHARGED=4
uint8 phase
