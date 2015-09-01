# CommonCalibrationStateMagnetoCalibrationStateChanged
# auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/common_commands.xml
# Date: 2015-08-31 18:59:03.288589
# Generator: generate.py @ f74e92a
# Do not modify this file by hand. Check scripts/meta folder for generator files.
#
# SDK Comment: Sent when the state of the magneto calibration has changed

Header header

# State of the x axis (roll) calibration : 1 if calibration is done, 0 otherwise
uint8 xAxisCalibration
# State of the y axis (pitch) calibration : 1 if calibration is done, 0 otherwise
uint8 yAxisCalibration
# State of the z axis (yaw) calibration : 1 if calibration is done, 0 otherwise
uint8 zAxisCalibration
# 1 if calibration has failed, 0 otherwise. If this arg is 1, consider all previous arg as 0
uint8 calibrationFailed
