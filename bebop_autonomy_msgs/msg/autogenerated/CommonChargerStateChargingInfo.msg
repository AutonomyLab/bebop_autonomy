# CommonChargerStateChargingInfo
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-08-31 18:59:03.288589
# Generator: generate.py @ f74e92a
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Information of the charge.

Header header

# The current charging phase.
uint8 phase_UNKNOWN=0
uint8 phase_CONSTANT_CURRENT_1=1
uint8 phase_CONSTANT_CURRENT_2=2
uint8 phase_CONSTANT_VOLTAGE=3
uint8 phase_CHARGED=4
uint8 phase_DISCHARGING=5
uint8 phase
# The charge rate. If phase is DISCHARGING, refers to the last charge.
uint8 rate_UNKNOWN=0
uint8 rate_SLOW=1
uint8 rate_MODERATE=2
uint8 rate_FAST=3
uint8 rate
# The charging intensity, in dA. (12dA = 1,2A) ; If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.
uint8 intensity
# The full charging time estimated, in minute. If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.
uint8 fullChargingTime
