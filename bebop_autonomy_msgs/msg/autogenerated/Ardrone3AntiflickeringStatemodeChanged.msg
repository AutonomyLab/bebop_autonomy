# Ardrone3AntiflickeringStatemodeChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
# Date: 2015-09-02
# Generator: generate.py @ ebb47cc
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Anti flickering mode

Header header

# Mode of the anti flickering functionnality
uint8 mode_auto=0  # Anti flickering based on the electric frequency previously sent
uint8 mode_FixedFiftyHertz=1  # Anti flickering based on a fixed frequency of 50Hz
uint8 mode_FixedSixtyHertz=2  # Anti flickering based on a fixed frequency of 60Hz
uint8 mode
