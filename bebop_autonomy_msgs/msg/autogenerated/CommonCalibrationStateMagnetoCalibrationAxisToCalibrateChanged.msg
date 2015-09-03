# CommonCalibrationStateMagnetoCalibrationAxisToCalibrateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-09-02
# Generator: generate.py @ 6ca12b4
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Event sent by a product to inform about the axis to calibrate

Header header

# The axis to calibrate
uint8 axis_xAxis=0  # If the current calibration axis should be the x axis
uint8 axis_yAxis=1  # If the current calibration axis should be the y axis
uint8 axis_zAxis=2  # If the current calibration axis should be the z axis
uint8 axis_none=3  # If none of the axis should be calibrated
uint8 axis
