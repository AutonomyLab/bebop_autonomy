# Ardrone3AccessoryStateConnectedAccessories
# auto-generated from up stream XML files at
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml
# To check upstream commit hash, refer to last_build_info file
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: List of all connected accessories. This event presents the list of all connected accessories. To actually use the component, use the component dedicated feature.

Header header

# Id of the accessory for the session.
uint8 id
# Accessory type
uint8 accessory_type_sequoia=0  # Parrot Sequoia (multispectral camera for agriculture)
uint8 accessory_type_unknownaccessory_1=1  # UNKNOWNACCESSORY_1 camera (thermal+rgb camera)
uint8 accessory_type
# Unique Id of the accessory. This id is unique by accessory_type.
string uid
# Software Version of the accessory.
string swVersion
