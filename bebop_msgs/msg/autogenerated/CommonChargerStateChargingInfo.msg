# CommonChargerStateChargingInfo
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Charging information.

Header header

# The current charging phase.
uint8 phase_UNKNOWN=0  # The charge phase is unknown or irrelevant.
uint8 phase_CONSTANT_CURRENT_1=1  # First phase of the charging process. The battery is charging with constant current.
uint8 phase_CONSTANT_CURRENT_2=2  # Second phase of the charging process. The battery is charging with constant current, with a higher voltage than the first phase.
uint8 phase_CONSTANT_VOLTAGE=3  # Last part of the charging process. The battery is charging with a constant voltage.
uint8 phase_CHARGED=4  # The battery is fully charged.
uint8 phase_DISCHARGING=5  # The battery is discharging; Other arguments refers to the last charge.
uint8 phase
# The charge rate. If phase is DISCHARGING, refers to the last charge.
uint8 rate_UNKNOWN=0  # The charge rate is not known.
uint8 rate_SLOW=1  # Slow charge rate.
uint8 rate_MODERATE=2  # Moderate charge rate.
uint8 rate_FAST=3  # Fast charge rate.
uint8 rate
# The charging intensity, in dA. (12dA = 1,2A) ; If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.
uint8 intensity
# The full charging time estimated, in minute. If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.
uint8 fullChargingTime
