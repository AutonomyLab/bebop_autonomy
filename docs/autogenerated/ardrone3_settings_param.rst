 .. Ardrone3_settings_param.rst
 .. auto-generated from https://raw.githubusercontent.com/Parrot-Developers/libARCommands/7e2f55fafcd45ba2380ca2574a08b7359c005f47/Xml/ARDrone3_commands.xml
 .. Date: 2015-09-07
 .. Do not modify this file by hand. Check scripts/meta folder for generator files.

*****************************************************************************************
List of Ardrone3 Setting and Corresponding ROS Parameter
*****************************************************************************************

pilotingsettings
-----------------------------------------------------------------------------------------
Piloting Settings commands

PilotingSettingsMaxAltitudeCurrent
  Current altitude max in m
PilotingSettingsMaxTiltCurrent
  Current tilt max in degree
PilotingSettingsAbsolutControlOn
  1 to enable, 0 to disable
PilotingSettingsMaxDistanceValue
  Current max distance in meter
PilotingSettingsNoFlyOverMaxDistanceShouldnotflyover
  1 if the drone cant fly further than max distance, 0 if no limitation on the drone should be done

speedsettings
-----------------------------------------------------------------------------------------
Speed Settings commands

SpeedSettingsMaxVerticalSpeedCurrent
  Current max vertical speed in m/s
SpeedSettingsMaxRotationSpeedCurrent
  Current max rotation speed in degree/s
SpeedSettingsHullProtectionPresent
  1 if present, 0 if not present
SpeedSettingsOutdoorOutdoor
  1 if outdoor flight, 0 if indoor flight

networksettings
-----------------------------------------------------------------------------------------
Network settings commands

NetworkSettingsWifiSelectionType
  The type of wifi selection (auto, manual)
NetworkSettingsWifiSelectionBand
  The allowed band(s) : 2.4 Ghz, 5 Ghz, or all
NetworkSettingsWifiSelectionChannel
  The channel (not used in auto mode)

settings
-----------------------------------------------------------------------------------------
Settings commands


picturesettings
-----------------------------------------------------------------------------------------
Photo settings chosen by the user


gpssettings
-----------------------------------------------------------------------------------------
GPS settings

GPSSettingsHomeTypeType
  The type of the home position
GPSSettingsReturnHomeDelayDelay
  Delay in second

