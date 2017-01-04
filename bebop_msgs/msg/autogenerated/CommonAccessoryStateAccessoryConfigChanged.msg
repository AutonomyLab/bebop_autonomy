# CommonAccessoryStateAccessoryConfigChanged
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Accessory config.

Header header

# Accessory configuration reported by firmware.
uint8 newAccessory_UNCONFIGURED=0  # No accessory configuration set. Controller needs to set one.
uint8 newAccessory_NO_ACCESSORY=1  # No accessory.
uint8 newAccessory_STD_WHEELS=2  # Standard wheels
uint8 newAccessory_TRUCK_WHEELS=3  # Truck wheels
uint8 newAccessory_HULL=4  # Hull
uint8 newAccessory_HYDROFOIL=5  # Hydrofoil
uint8 newAccessory_IN_PROGRESS=6  # Configuration in progress.
uint8 newAccessory
# Error code.
uint8 error_OK=0  # No error. Accessory config change successful.
uint8 error_UNKNOWN=1  # Cannot change accessory configuration for some reason.
uint8 error_FLYING=2  # Cannot change accessory configuration while flying.
uint8 error
